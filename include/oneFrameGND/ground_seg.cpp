#include "ground_seg.h"

/*
    @brief Compare function to sort points. Here use z axis.
    @return z-axis accent
*/
bool point_cmp(pcl::PointXYZI a, pcl::PointXYZI b)
{
	return a.z < b.z;
}

PlaneGroundFilter::PlaneGroundFilter( )
{
	g_seeds_pc = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	g_ground_pc = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	g_not_ground_pc = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	g_all_pc = pcl::PointCloud<SLRPointXYZIRL>::Ptr(new pcl::PointCloud<SLRPointXYZIRL>);
    final_no_ground = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	readParam();
}

PlaneGroundFilter::~PlaneGroundFilter() {}



void PlaneGroundFilter::clip_above(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
								   const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
	pcl::ExtractIndices<pcl::PointXYZI> cliper;
	
	cliper.setInputCloud(in);
	pcl::PointIndices indices;
#pragma omp for
	for (size_t i = 0; i < in->points.size(); i++)
	{
		if (in->points[i].z > clip_height_)
		{
			indices.indices.push_back(i);
		}
	}
	cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
	cliper.setNegative(true); //ture to remove the indices
	cliper.filter(*out);
}

void PlaneGroundFilter::remove_close_far_pt(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
											const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
	pcl::ExtractIndices<pcl::PointXYZI> cliper;
	
	cliper.setInputCloud(in);
	pcl::PointIndices indices;
#pragma omp for
	for (size_t i = 0; i < in->points.size(); i++)
	{
		double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);
		
		if ((distance < min_distance_) || (distance > max_distance_))
		{
			indices.indices.push_back(i);
		}
	}
	cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
	cliper.setNegative(true); //ture to remove the indices
	cliper.filter(*out);
}

/*
    @brief The function to estimate plane model. The
    model parameter `normal_` and `d_`, and `th_dist_d_`
    is set here.
    The main step is performed SVD(UAV) on covariance matrix.
    Taking the sigular vector in U matrix according to the smallest
    sigular value in A, as the `normal_`. `d_` is then calculated
    according to mean ground points.

    @param g_ground_pc:global ground pointcloud ptr.
    
*/
void PlaneGroundFilter::estimate_plane_(void)
{
	// Create covarian matrix in single pass.
	// TODO: compare the efficiency.
	Eigen::Matrix3f cov;
	Eigen::Vector4f pc_mean;
	pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
	// Singular Value Decomposition: SVD
	JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
	// use the least singular vector as normal
	normal_ = (svd.matrixU().col(2));
	// mean ground seeds value
	Eigen::Vector3f seeds_mean = pc_mean.head<3>();
	
	// according to normal.T*[x,y,z] = -d
	d_ = -(normal_.transpose() * seeds_mean)(0, 0);
	// set distance threhold to `th_dist - d`
	th_dist_d_ = th_dist_ - d_;
	
	// return the equation parameters
}

/*
    @brief Extract initial seeds of the given pointcloud sorted segment.
    This function filter ground seeds points accoring to heigt.
    This function will set the `g_ground_pc` to `g_seed_pc`.
    @param p_sorted: sorted pointcloud
    
    @param ::num_lpr_: num of LPR points
    @param ::th_seeds_: threshold distance of seeds
    @param ::
*/
void PlaneGroundFilter::extract_initial_seeds_(const pcl::PointCloud<pcl::PointXYZI> &p_sorted)
{
	// LPR is the mean of low point representative代表
	double sum = 0;
	int cnt = 0;
	// Calculate the mean height value.
	for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++)
	{
		sum += p_sorted.points[i].z;
		cnt++;
	}
	double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0
	g_seeds_pc->clear();
	// iterate pointcloud, filter those height is less than lpr.height+th_seeds_
	for (int i = 0; i < p_sorted.points.size(); i++)
	{
		if (p_sorted.points[i].z < lpr_height + th_seeds_)
		{
			g_seeds_pc->points.push_back(p_sorted.points[i]);
		}
	}
	// return seeds points
}

void PlaneGroundFilter::post_process(const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cliped_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	clip_above(in, cliped_pc_ptr);
	pcl::PointCloud<pcl::PointXYZI>::Ptr remove_close(new pcl::PointCloud<pcl::PointXYZI>);
	remove_close_far_pt(cliped_pc_ptr, out);
}

void PlaneGroundFilter::point_cb(const pcl::PointCloud<pcl::PointXYZI> laserCloudIn1)
{
	//清空最终结果
	g_ground_pc->clear();
	final_no_ground->clear();
	g_all_pc->clear();
	// 1.Msg to pointcloud
	pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
	pcl::PointCloud<pcl::PointXYZI> laserCloudIn_org;
	laserCloudIn.resize(laserCloudIn1.size());
	laserCloudIn_org.resize((laserCloudIn1.size()));
	for (int k = 0; k < laserCloudIn1.size(); ++k) {
		laserCloudIn.points[k].x = laserCloudIn1.points[k].x;
		laserCloudIn.points[k].y = laserCloudIn1.points[k].y;
		laserCloudIn.points[k].z = laserCloudIn1.points[k].z;
		laserCloudIn.points[k].intensity = laserCloudIn1.points[k].intensity;
	}
	laserCloudIn_org = laserCloudIn;
	
	// For mark ground points and hold all points
	SLRPointXYZIRL point;
	
	for (size_t i = 0; i < laserCloudIn.points.size(); i++)
	{
		point.x = laserCloudIn.points[i].x;
		point.y = laserCloudIn.points[i].y;
		point.z = laserCloudIn.points[i].z;
		point.intensity = laserCloudIn.points[i].intensity;
	 
		point.label = 0u; // 0 means uncluster
		g_all_pc->points.push_back(point);
	}
	//std::vector<int> indices;
	//pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn,indices);
	// 2.Sort on Z-axis value.
	sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_cmp);
	// 3.Error point removal
	// As there are some error mirror reflection under the ground,
	// here regardless point under 2* sensor_height
	// Sort point according to height, here uses z-axis in default
	pcl::PointCloud<pcl::PointXYZI>::iterator it = laserCloudIn.points.begin();
	for (int i = 0; i < laserCloudIn.points.size(); i++)
	{
		if (laserCloudIn.points[i].z < -1.5 * sensor_height_)
		{
			it++;
		}
		else
		{
			break;
		}
	}
	laserCloudIn.points.erase(laserCloudIn.points.begin(), it);
	// 4. Extract init ground seeds.
	extract_initial_seeds_(laserCloudIn);
	g_ground_pc = g_seeds_pc;
	// 5. Ground plane fitter mainloop
	for (int i = 0; i < num_iter_; i++)
	{
		estimate_plane_();
		g_ground_pc->clear();
		g_not_ground_pc->clear();
		
		//pointcloud to matrix
		MatrixXf points(laserCloudIn_org.points.size(), 3);
		int j = 0;
		for (auto p : laserCloudIn_org.points)
		{
			points.row(j++) << p.x, p.y, p.z;
		}
		// ground plane model
		VectorXf result = points * normal_;
		// threshold filter
		for (int r = 0; r < result.rows(); r++)
		{
			if (result[r] < th_dist_d_)
			{
				g_all_pc->points[r].label = 1u; // means ground
				g_ground_pc->points.push_back(laserCloudIn_org[r]);
			}
			else
			{
				g_all_pc->points[r].label = 0u; // means not ground and non clusterred
				g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
			}
		}
	}
 
	post_process(g_not_ground_pc, final_no_ground);
	
	// ROS_INFO_STREAM("origin: "<<g_not_ground_pc->points.size()<<" post_process: "<<final_no_ground->points.size());
	
	
}

void PlaneGroundFilter::readParam() {
 
		//1.读取yaml参数
	YAML::Node node = YAML::LoadFile("/media/echo/DataDisc/3_program/mapping/cfg/groundSegParm.yaml");
	clip_height_ = node["clip_height_"].as<double>();
	sensor_height_ = node["sensor_height_"].as<double>();
	min_distance_ = node["min_distance_"].as<double>();
	max_distance_ = node["max_distance_"].as<double>();
	sensor_model_ = node["sensor_model_"].as<int>();
	num_iter_ = node["num_iter_"].as<int>();
	num_lpr_ = node["num_lpr_"].as<int>();
	th_seeds_ = node["th_seeds_"].as<double>();
	th_dist_ = node["th_dist_"].as<double>();
 
}

//
// Created by echo on 2020/12/9.
//

#include "point_to_gridmap.h"

void point_to_gridmap::calcPcBoundary(double &xMax, double &yMax, double &xMin, double &yMin) {
	for (size_t i = 0; i < currentPC.size(); i++) {
		double x = currentPC.points[i].x;
		double y = currentPC.points[i].y;
		if (xMax < x) {
			xMax = x;
		}
		if (xMin > x) {
			xMin = x;
		}
		if (yMax < y) {
			yMax = y;
		}
		if (yMin > y) {
			yMin = y;
		}
	}
}

void
point_to_gridmap::calcSurfaceNormals(pcl::PointCloud<pcl::PointXYZI> cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
	currentPC = cloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	*cloud_ptr = cloud;
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	ne.setInputCloud(cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.3);
	ne.compute(*normals);
}

void point_to_gridmap::populateMap(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, std::vector<int> &map, double xMax,
								   double yMax, double xMin, double yMin, double cellResolution, int xCells,int yCells) {
	double deviation = 1;
	std::vector<std::vector<int> > map_n;
	
	int the_size_of_map;
	the_size_of_map = (xMax-xMin)*(yMax-yMin)/(cellResolution*cellResolution);
	
	int conditions=2;
	map_n.clear();
	map_n.resize(conditions);
	
	std::cout<<" the averanger z axis height is :"<< 1<< std::endl;
	
	for(int j =0; j < conditions; j++)
	{
		map_n[j].resize(the_size_of_map); // whether it is not big enough to put the point cloud in？currentPC->size()/cellResolution
	}
	
	std::cout<< "test point 1"<<std::endl;
	//int x1 = currentPC->size()/(cellResolution*cellResolution);
	for (size_t i = 0; i < currentPC.size(); i++)
	{
		double x = currentPC.points[i].x;
		double y = currentPC.points[i].y;
		double z = cloud_normals->points[i].normal_z;
		double z_axis = currentPC.points[i].z;
		
		double phi = acos(fabs(z));
		int xCell, yCell;
		// std::cout<< i <<" i,  the_size_.of_map: "<<the_size_of_map<<"  the place to write in map_n "<<yCell * xCells + xCell<<std::endl;
		if ( 1) { //TODO implement cutoff height!!!       z_axis < threshold_z1 + cutoff_height
			xCell = (int) ((x - xMin) / cellResolution);
			yCell = (int) ((y - yMin) / cellResolution);
			
			if (phi > deviation)
			{
//				if(z_axis <  0.3 + 1)               //TODO this part is to calculate the height of ground               threshold_z1 +
				if(1)
				{
					map_n[0][yCell * xCells + xCell] ++;
				}else
				{
					map_n[1][yCell * xCells + xCell] ++;
				}
				map[yCell * xCells + xCell]++;
			}
			if(map[yCell * xCells + xCell]==0)
			{
				map[yCell * xCells + xCell]=1;
				// map[yCell * xCells + xCell]--;
			}
		}
	}
	std::cout<< "test point 2"<<std::endl;
	for(int m =0; m < currentPC.size(); m++)
	{
		double x1 = currentPC.points[m].x;
		double y1 = currentPC.points[m].y;
		int xCell1 = (int) ((x1 - xMin) / cellResolution);
		int yCell1 = (int) ((y1 - yMin) / cellResolution);
		//std::cout<<" current m:"<<m<<std::endl;
		if(map_n[0][yCell1 * xCells + xCell1] > map_n[1][yCell1 * xCells + xCell1] && map_n[0][yCell1 * xCells + xCell1] >= 15) //TODO  here is calculate the greeen block
		{
			//TODO m is not poper with yCell * xCells + xCell
			map[yCell1 * xCells + xCell1] = -1;
		}
	}
}

void point_to_gridmap::genOccupancyGrid(std::vector<signed char> &ocGrid, std::vector<int> &countGrid, int size,int xcells) {
	std::cout<<"genOccupancyGrid"<<std::endl;
	for (int i = 0; i < size; i++) {
		if (countGrid[i] < 1 && countGrid[i]>0)   //10 is not changeds
		{
			ocGrid[i] = 0;
		} else if (countGrid[i] >= 2 &&countGrid[i] <= 4)
		{
			ocGrid[i] = 100;
		} else if (countGrid[i] == 0)
		{
			if(countGrid[i+1] == 0 && countGrid[i-1] == 0){
				ocGrid[i] = -1;//2018.1.15 not 100 used to be -1
			} else{
				ocGrid[i] = 0;
			}
			
		} else if (countGrid[i] < 20 && countGrid[i] >= 4)
		{
			ocGrid[i] = 200; // TODO Should be -1 200
		} else if (countGrid[i] >= 20)
		{
			ocGrid[i] = 100; // TODO Should be -1 150
		} else if (countGrid[i] = -1) //2018.1.15 not 100 used to be -1
		{
			if(countGrid[i+1] == -1 || countGrid[abs(i-1)] == -1 || countGrid[abs(i-xcells)]== -1 || countGrid[i+xcells] == -1){
				ocGrid[i] = 100;//125
			}else{
				ocGrid[i] = 0;
			}
		}
	}
	std::cout<<" genOccupancyGrid finshed:"<< std::endl;
}

void point_to_gridmap::updateGrid(nav_msgs::OccupancyGridPtr grid, double cellRes, int xCells, int yCells, double originX,
							 double originY, std::vector<signed char> *ocGrid) {
	grid->header.seq++;
	grid->header.stamp.sec = ros::Time::now().sec;
	grid->header.stamp.nsec = ros::Time::now().nsec;
	grid->info.map_load_time = ros::Time::now();
	grid->info.resolution = cellRes;
	grid->info.width = xCells;
	grid->info.height = yCells;
	grid->info.origin.position.x = originX;
	grid->info.origin.position.y = originY;
	grid->data = *ocGrid;
 
}

void point_to_gridmap::groundVoxelMap(pcl::PointCloud<pcl::PointXYZI> cloud, nav_msgs::OccupancyGridPtr grid) {

}

void point_to_gridmap::initGrid(nav_msgs::OccupancyGridPtr grid) {
	grid->header.seq = 1;
	grid->header.frame_id = "/map";
	grid->info.origin.position.z = 0;
	grid->info.origin.orientation.w = 1;
	grid->info.origin.orientation.x = 0;
	grid->info.origin.orientation.y = 0;
	grid->info.origin.orientation.z = 0;
	
}

void point_to_gridmap::saveGridasPNG(nav_msgs::OccupancyGridPtr grid, std::string image_path) {
	std::cout<<"saveGridasPNG"<<std::endl;
	grid_map::GridMap map;
	cv_bridge::CvImage image;
	grid_map::GridMapRosConverter::fromOccupancyGrid(*grid, "map",map);
	grid_map::GridMapRosConverter::toCvImage(map,"map", sensor_msgs::image_encodings::RGB8, image);
	bool success = cv::imwrite(image_path.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
	std::cout<<success<<std::endl;
}



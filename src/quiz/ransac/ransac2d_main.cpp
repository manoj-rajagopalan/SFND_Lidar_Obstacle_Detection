/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <random>

#include <Eigen/Eigen>

#include "../../render/render.h"
#include <unordered_set>
// #include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
// #include "../../processPointClouds.cpp"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "ransac2d.hpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	// ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	// return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");

	const std::string file = "../../../sensors/data/pcd/simpleHighway.pcd";
	
	typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

inline
Eigen::Vector3d operator - (const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
	return Eigen::Vector3d(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline bool operator == (const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

#if 0 //- aborted attempt, see ransac2d.cpp

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int maxIterations,
							   float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// MANOJ: Fill in this function
	std::uniform_int_distribution<int> rnd_idx(0, cloud->points.size()-1);
	std::mt19937 rnd_gen;

	// For max iterations 
	for(int iter = 0; iter < maxIterations; ++iter) {
		// Randomly sample subset and fit line
		using P = pcl::PointXYZ;
		const int idx0 = rnd_idx(rnd_gen);
		const P& v0 = cloud->points.at(idx0);
		const Eigen::Vector2d p0{v0.x, v0.y};
		int idx1 = idx0;
		P v1 = v0;
		while(idx1 == idx0 || v1 == v0) {
			idx1 = rnd_idx(rnd_gen);
			v1 = cloud->points.at(idx1);
		}
		const Eigen::Vector2d p1{v1.x, v1.y};
		/*
		// Eqn of line is Ax + By + C = 0, where
		const double A = v1.y - v0.y;
		const double B = v0.x - v1.x;
		const double C = v1.x*v0.y - v0.x*v1.y;
		const double normal_norm = std::sqrt(A*A + B*B);
		*/

		const Eigen::Vector2d l01 = p1 - p0;
		Eigen::Vector2d unit_normal{-l01[1], l01[0]};
		unit_normal.normalize();

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		std::vector<int> inliers;
		for(int n = 0; n < cloud->points.size(); ++n) {
			if(n == idx0 || n == idx1) continue;
			const auto& v = cloud->points[n];
			const Eigen::Vector2d p{v.x, v.y};
			// const double dist = std::abs(A*p.x+B*p.y+C) / normal_norm;
			const double dist = std::abs(unit_normal.dot(p - p0));
			if(dist < distanceTol) {	
				inliers.push_back(n);
			}
		}
		
		if(inliers.size() > inliersResult.size()) {
			inliersResult.clear();
			std::copy(inliers.cbegin(),
			          inliers.cend(),
					  std::inserter(inliersResult, inliersResult.begin()));
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}
#endif



int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// MANOJ: Change the max iteration and distance tolerance arguments for Ransac function
	const std::vector<int> inliers = RansacPlane<pcl::PointXYZ>(cloud, 100, 0.2f);
	//- std::cout << "inliers.size = " << inliers.size() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	int index = 0;
	for(const int inlier_index : inliers) {
		for(; index < inlier_index; ++index) {
			cloudOutliers->points.push_back(cloud->points[index]);
		}
		cloudInliers->points.push_back(cloud->points[index]);
		++index;
	}
	for(; index < cloud->points.size(); index++) {
		cloudOutliers->points.push_back(cloud->points[index]);
	}

#if 0 //-
	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
#endif

	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}

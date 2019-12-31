// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

#include "quiz/cluster/cluster.hpp"
#include "quiz/ransac/ransac2d.hpp"

#define MANOJS_CODE

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                        float filterRes,
                                        Eigen::Vector4f minPoint,
                                        Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Manoj:: Fill in the function to do voxel grid point reduction and region based filtering

    // downsample
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(filterRes, filterRes, filterRes);
    typename pcl::PointCloud<PointT>::Ptr downsampled_cloud(new pcl::PointCloud<PointT>{});
    voxel_grid.filter(*downsampled_cloud);

    // crop to region of interest
    pcl::CropBox<PointT> roi_filter(true);
    roi_filter.setMin(minPoint);
    roi_filter.setMax(maxPoint);
    roi_filter.setInputCloud(downsampled_cloud);
    typename pcl::PointCloud<PointT>::Ptr roi_filtered_cloud(new pcl::PointCloud<PointT>{});
    roi_filter.filter(*roi_filtered_cloud);

    // remove roof top
    pcl::CropBox<PointT> roof_filter(true);
    roof_filter.setMin(Eigen::Vector4f{-1.5f, -1.7f, -1.0f, 1.0f});
    roof_filter.setMax(Eigen::Vector4f{2.6f, 1.7f, -0.4f, 1.0f});
    roof_filter.setInputCloud(roi_filtered_cloud);
    std::vector<int> roof_inlier_indices;
    roof_filter.filter(roof_inlier_indices);

    pcl::PointIndices::Ptr roof_inlier_point_indices(new pcl::PointIndices);
    for(auto i : roof_inlier_indices) {
        roof_inlier_point_indices->indices.push_back(i);
    }

    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(roi_filtered_cloud);
    extractor.setIndices(roof_inlier_point_indices);
    extractor.setNegative(true);
    extractor.filter(*roi_filtered_cloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
              << " and reduced from " << cloud->size() << " to " << roi_filtered_cloud->size() << std::endl;

    return roi_filtered_cloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers,
                                               typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // MANOJ: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    typename pcl::PointCloud<PointT>::Ptr road{new pcl::PointCloud<PointT>};
    extract.filter(*road);
    extract.setNegative(true);
    typename pcl::PointCloud<PointT>::Ptr obstacles{new pcl::PointCloud<PointT>};
    extract.filter(*obstacles);
    std::pair<typename pcl::PointCloud<PointT>::Ptr,
              typename pcl::PointCloud<PointT>::Ptr> segResult(road, obstacles);    
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                             int maxIterations,
                                             float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

#ifdef MANOJS_CODE
    inliers->indices = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);

#else // ! MANOJS_CODE

	pcl::ModelCoefficients::Ptr coeffs{new pcl::ModelCoefficients};

    // MANOJ: Fill in this function to find inliers for the cloud.

    pcl::SACSegmentation<PointT> seg;
    seg.setInputCloud(cloud);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.segment(*inliers, *coeffs);
#endif // MANOJS_CODE

    if(inliers->indices.size() == 0) {
        std::cerr << "Could not segment point cloud"    << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr,
              typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

#ifdef MANOJS_CODE
template <typename PointT>
static
void proximity(typename pcl::PointCloud<PointT>::Ptr &cloud,
               const KdTree &tree,
			   float distanceTol,
			   int n,
               int maxSize,
			   typename pcl::PointCloud<PointT>::Ptr &new_cluster,
			   std::vector<bool>& processed)
{
    if(new_cluster->size() >= maxSize) {
        return;
    }
	processed[n] = true;
	new_cluster->push_back(cloud->at(n));
	std::vector<int> nearby = tree.search({cloud->at(n).x, cloud->at(n).y}, distanceTol);
	for(const int nearby_id : nearby) {
		if(!processed[nearby_id]) {
			proximity<PointT>(cloud, tree, distanceTol, nearby_id, maxSize, new_cluster, processed);
		}
	}
}
#endif

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                       float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> processed(cloud->size(), false);

#ifdef MANOJS_CODE
    KdTree tree;
    for(int i = 0; i < int(cloud->size()); ++i) {
        tree.insert({cloud->at(i).x, cloud->at(i).y}, i);
    }

	for(int n = 0; n < cloud->size(); ++n) {
		if(!processed[n]) {
			clusters.emplace_back(new pcl::PointCloud<PointT>());
			auto& new_cluster = clusters.back();
			proximity<PointT>(cloud, tree, clusterTolerance, n, maxSize, new_cluster, processed);
            if(new_cluster->size() < minSize) {
                clusters.pop_back();
            }
		}
	}

#else // ! MANOJS_CODE
    // Manoj:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr kd_tree(new pcl::search::KdTree<PointT>);
    kd_tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(kd_tree);
    ec.setInputCloud(cloud);
    std::vector<pcl::PointIndices> point_indices_cluster;
    ec.extract(point_indices_cluster);
    std::cerr << point_indices_cluster.size() << " Euclidean clusters extracted" << std::endl;

    for(const auto& point_indices : point_indices_cluster) {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for(auto point_iter = point_indices.indices.cbegin();
            point_iter != point_indices.indices.cend();
            ++point_iter) {
            cluster->push_back(cloud->points[*point_iter]);
        }
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }
#endif // MANOJS_CODE

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <array>
#include <memory>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


constexpr int kNumColors = 10;
constexpr double kColorDelta = 1.0 / kNumColors;
std::vector<Color> g_clusterColors;

void initClusterColors()
{
    for(int n = 0; n < kNumColors; ++n) {
        g_clusterColors.emplace_back(n * kColorDelta,
                                     ((kNumColors/2 + n) % kNumColors) * kColorDelta,
                                     (1.0 - n * kColorDelta));
    }
}

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor
    std::unique_ptr<Lidar> lidar(new Lidar(cars, 0.0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> process_pt_clouds;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
              pcl::PointCloud<pcl::PointXYZ>::Ptr>
        road_and_obstacles = process_pt_clouds.SegmentPlane(cloud, 3, 0.2);
    renderPointCloud(viewer, road_and_obstacles.first, "road", Color(0,1,0));
    renderPointCloud(viewer, road_and_obstacles.second, "obstacles", Color(1    ,0,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters =
        process_pt_clouds.Clustering(road_and_obstacles.second, 1.0, 2, 25000);
    std::cerr << clusters.size() << " Euclidean clusters returned\n";
    int cluster_idx = 0;
    constexpr double kStep = 0.2;
    for(const auto& cluster : clusters) {
        renderPointCloud(viewer, cluster,
                         "cluster" + std::to_string(cluster_idx),
                         Color(cluster_idx * kStep, 1, 1.0 - (cluster_idx * kStep)));   
        const Box bbox = process_pt_clouds.BoundingBox(cluster);
        renderBox(viewer, bbox, cluster_idx);
        ++cluster_idx;
    }

    // renderRays(viewer, lidar->position, cloud);
    //- renderPointCloud(viewer, cloud, "pt cloud", Color(0.0, 1.0, 0.0));
    
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    //- // Load PCD
    //- ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>;
    //- pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    // Filter the point cloud
    const float long_horizon = 40.0f;
    const float left_horizon = 8.0f;
    const float right_horizon = 4.0f;
    const float vert_horizon = 2.0f;
    const Eigen::Vector4f crop_min{-long_horizon, -right_horizon, -vert_horizon, 1.0f};
    const Eigen::Vector4f crop_max{ long_horizon,  left_horizon,  vert_horizon, 1.0f};
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.1f, crop_min, crop_max);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = inputCloud;
    //? renderPointCloud(viewer, filteredCloud, "filtered cloud");

    // Segment into road and obstacles
    constexpr int kSegMaxIters = 100;
    constexpr double kSegDistanceThreshold = 0.2;
    auto seg_result = pointProcessorI->SegmentPlane(filteredCloud, kSegMaxIters, kSegDistanceThreshold);
    pcl::PointCloud<pcl::PointXYZI>::Ptr road_cloud = seg_result.first;
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles_cloud = seg_result.second;

    // Cluster the obstacles
    constexpr double kClusterTol = 1.0;
    constexpr int kClusterMinSize = 100;
    constexpr int kClusterMaxSize = 25000;
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacle_clusters =
        pointProcessorI->Clustering(obstacles_cloud, kClusterTol, kClusterMinSize, kClusterMaxSize);
 
    // Draw bounding box and render
    int clusterIdx = 0;
    for(auto ob_cluster : obstacle_clusters) {
        const Box bbox = pointProcessorI->BoundingBox(ob_cluster);
        renderPointCloud(viewer, ob_cluster, "object " + std::to_string(clusterIdx), g_clusterColors[clusterIdx]);
        renderBox(viewer, bbox, clusterIdx, g_clusterColors[kNumColors - clusterIdx - 1], 0.5f);
        ++clusterIdx;
    }
    renderPointCloud(viewer, road_cloud, "road", Color(0.0, 1.0, 0.0));
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    initClusterColors();
    // simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIter = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        auto startTime = std::chrono::steady_clock::now();

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI.loadPcd(streamIter->string());
        cityBlock(viewer, &pointProcessorI, inputCloudI);

        ++streamIter;
        if(streamIter == stream.end()) {
            streamIter = stream.begin();
        }

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "--- Event loop took " << elapsedTime.count() << " milliseconds" << std::endl;

        viewer->spinOnce ();
    } 
}
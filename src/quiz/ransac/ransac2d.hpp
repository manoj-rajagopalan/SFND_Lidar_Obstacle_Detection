#ifndef RANSAC2D_HPP_
#define RANSAC2D_HPP_

#include <unordered_set>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    int maxIterations,
							        float distanceTol);

#endif // RANSAC2D_HPP_

#ifndef CLUSTER_HPP_
#define CLUSTER_HPP_

#include <vector>

#include "kdtree.h"

std::vector<std::vector<int>>
euclideanCluster(const std::vector<std::vector<float>>& points,
                 KdTree* tree,
				 float distanceTol);

#endif // CLUSTER_HPP_

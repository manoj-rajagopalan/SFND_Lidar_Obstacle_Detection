#ifndef RANSAC2D_HPP_
#define RANSAC2D_HPP_

#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#if 0 //-
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    int maxIterations,
							        float distanceTol);
#endif

template <typename PointT>
std::vector<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                             int maxIterations,
							 float distanceTol)
{
	srand(time(NULL));
	std::vector<int> inliersResult;
	std::uniform_int_distribution<int> rnd_idx(0, cloud->points.size());
	std::mt19937 rnd_gen;

	for(int iter = 0; iter < maxIterations; ++iter)
	{
		Eigen::Vector3d p0;
		Eigen::Vector3d normal;
		double normal_length = 0.0;

		while(normal_length < 1.0e-4) {
			using Eigen::Vector3d;
			const int i0 = rnd_idx(rnd_gen);
			const PointT& v0 = cloud->points[i0];
			const int i1 = rnd_idx(rnd_gen);
			const PointT& v1 = cloud->points[i1];
			const int i2 = rnd_idx(rnd_gen);
			const PointT& v2 = cloud->points[i2];
			const Eigen::Vector3d v01 = Vector3d(v1.x, v1.y, v1.z) - Vector3d(v0.x, v0.y, v0.z);
			const Eigen::Vector3d v02 = Vector3d(v2.x, v2.y, v2.z) - Vector3d(v0.x, v0.y, v0.z);
			normal = v01.cross(v02);
			normal_length = normal.norm();
			p0 = {v0.x, v0.y, v0.z};
		}
		normal = normal / normal_length; // make unit vec
		std::vector<int> inliers;
		for (int n = 0; n < cloud->points.size(); ++n) {
			const PointT& v = cloud->points[n];
			const Eigen::Vector3d p(v.x, v.y,  v.z);
			const double distance_to_plane = std::abs(normal.dot(p - p0));
			if(distance_to_plane < distanceTol) {
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

	return inliersResult;
}

#endif // RANSAC2D_HPP_

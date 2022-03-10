#ifndef POINTCLOUDREGISTRATION_H
#define POINTCLOUDREGISTRATION_H

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <vector>      
#include <queue>
struct keyPoint{
    pcl::PointXYZRGBNormal u_l_b, u_l_f, u_r_b, u_r_f, d_l_b, d_l_f, d_r_b, d_r_f;
    pcl::PointXYZ min, max;
    pcl::PointXYZRGBNormal center_point;
    float cells;
    int number_of_points = 0;
    int cluster;
    int cl;
    float V;
    float x,y,z;
};                    

class PointCloudRegistration {

public:
    PointCloudRegistration();
    virtual ~PointCloudRegistration();
    Eigen::Matrix4f coarseAlignment3D(std::vector<keyPoint>& objects1, std::vector<keyPoint>& objects2);
    Eigen::Matrix4f fineAlignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target);
    bool getMatch();
    void filterFarPoints(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, float dis);
    float max_i = 0.0;
    std::vector<std::pair<int, int>> pairs;
    std::vector<int> container;
    float maximum = 0.0;
    float average = 0.0;
    float sum = 0.0;
    float counter = 0.0;
    Eigen::Matrix4f post_transform = Eigen::Matrix4f::Identity();
    bool goodMatch = false;
};

#endif
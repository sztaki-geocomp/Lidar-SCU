#include "Registration.h"
#include <math.h>
#include <stdlib.h>
#include <pcl/registration/icp.h>

PointCloudRegistration::PointCloudRegistration()
{
}

PointCloudRegistration::~PointCloudRegistration()
{
}

struct Acc
{
    int i = 0;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float rotz = 0.0f;
};

Eigen::Matrix4f PointCloudRegistration::fineAlignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_source(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_target(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (auto p:*source){
        //Pillar
		if(p.r == 255){
		    obj_source->points.push_back(p);
	    }
    }
    for (auto p:*target){
        //Pillar
		if(p.r == 255){
		    obj_target->points.push_back(p);
	    }
    }
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    pcl::PointCloud<pcl::PointXYZRGB> source_out;
    // Set the input source and target
    icp.setInputSource (obj_source);
    icp.setInputTarget (obj_target);
    // Set the max correspondence distance to 50cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.5);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (10000);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-4);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (0.0001);
    // Perform the alignment
    icp.align(source_out);




    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore(7.0) << std::endl;
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    return icp.getFinalTransformation();
    // return;
}

void PointCloudRegistration::filterFarPoints(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, float dis)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal> tmp;
    for (auto &p : cloud.points)
    {
        if (sqrt(p.x * p.x + p.y * p.y) < dis)
            tmp.push_back(p);
    }
    cloud.clear();

    for (auto &p : tmp.points)
    {
        cloud.push_back(p);
    }
    // for(auto& p : cloud.points) {
    //     if( fabs(p.x) < 20 && fabs(p.y) < 30)
    //         tmp.push_back(p);
    // }
    // cloud.clear();

    // for(auto& p : tmp.points) {
    //     cloud.push_back(p);
    // }
}

bool PointCloudRegistration::getMatch(){
    return goodMatch;
}
Eigen::Matrix4f PointCloudRegistration::coarseAlignment3D(std::vector<keyPoint> &objects1, std::vector<keyPoint> &objects2)
{

    // Parameters
    float rot_step = 0.25f;
    float planar_resolution = 0.2f;
    float z_resolution = 2.0f; 
    float rot_max = 60.0f;     
    float planar_max = 12.0f;
    float x_max = 12.0f;
    float y_max = 12.0f; 
    float z_max = 2.0f; 
    goodMatch = false;

    // Accumulator dimensions
    int x_dim = (int)(x_max * 2.0f / planar_resolution);
    int y_dim = (int)(y_max * 2.0f / planar_resolution);
    int z_dim = (int)(z_max * 2.0f / z_resolution);
    int rot_dim_z = (int)(rot_max * 2.0f / rot_step);

    //std::cout<< "Acc dimensions:" << x_dim << " x " << y_dim << " x " << z_dim << " x " << rot_dim_z << std::endl;

    Acc ****transformsAcc = new Acc ***[rot_dim_z];
    for (int i = 0; i < rot_dim_z; ++i)
    {
        transformsAcc[i] = new Acc **[x_dim];
        for (int l = 0; l < x_dim; ++l)
        {
            transformsAcc[i][l] = new Acc *[y_dim];
            for (int m = 0; m < y_dim; ++m)
            {
                transformsAcc[i][l][m] = new Acc[z_dim];
            }
        }
    }

    float imax = -1, jmax = -1, kmax = -1, rotmax_z = -1;

    // down points
    float x_i3d_1, y_i3d_1, z_i3d_1, x_i3d_2, y_i3d_2, z_i3d_2, x_i3d_3, y_i3d_3, z_i3d_3, x_i3d_4, y_i3d_4, z_i3d_4;
    float x_mls_1, y_mls_1, z_mls_1, x_mls_2, y_mls_2, z_mls_2, x_mls_3, y_mls_3, z_mls_3, x_mls_4, y_mls_4, z_mls_4;
    // up points
    float x_i3d_5, y_i3d_5, z_i3d_5, x_i3d_6, y_i3d_6, z_i3d_6, x_i3d_7, y_i3d_7, z_i3d_7, x_i3d_8, y_i3d_8, z_i3d_8;
    float x_mls_5, y_mls_5, z_mls_5, x_mls_6, y_mls_6, z_mls_6, x_mls_7, y_mls_7, z_mls_7, x_mls_8, y_mls_8, z_mls_8;

    // transform
    float dx, dy, dz;

    // float ux_, uy_, uz_, ux__, uy__, uz__, ux___, uy___, uz___, ux2, uy2, uz2, udx, udy, udz;
    // float ux1_, uy1_, uz1_, ux1__, uy1__, uz1__, ux1___, uy1___, uz1___;
    // float ux2_, uy2_, uz2_, ux2__, uy2__, uz2__, ux2___, uy2___, uz2___;
    // float ux3_, uy3_, uz3_, ux3__, uy3__, uz3__, ux3___, uy3___, uz3___;

    int dx_res, dy_res, dz_res;
    int idx_x = 0, idx_y = 0, idx_z = 0, idx_rot_z = 0;

    bool mls = false;
    bool i3d = false;
    int cntVelo = 0;
    int cntMLS = 0;
    bool to_be_continued = false;
    pcl::PointXYZRGB bottom_corners[4];
    pcl::PointXYZRGB top_corners[4];

    for (int i = 0; i < objects1.size(); ++i)
    {
        //std::cout << " Velo object size: " <<objects1[i].number_of_points<<std::endl;
        cntMLS = 0;
        for (int j = 0; j < objects2.size(); ++j)
        {
            int vote_weigth = 1;
            {
                for (int rot_ = 0; rot_ < rot_dim_z; ++rot_)
                {
                    float degree_z = rot_step * (float)rot_ - rot_max;
                    // std::cout<<degree_z<<std::endl;
                    // down-left-back
                    bottom_corners[0].x = cos(DEG2RAD(degree_z)) * objects1[i].d_l_b.x - sin(DEG2RAD(degree_z)) * objects1[i].d_l_b.y;
                    bottom_corners[0].y = sin(DEG2RAD(degree_z)) * objects1[i].d_l_b.x + cos(DEG2RAD(degree_z)) * objects1[i].d_l_b.y;
                    bottom_corners[0].z = objects1[i].d_l_b.z;

                    // down-left-front
                    bottom_corners[1].x = cos(DEG2RAD(degree_z)) * objects1[i].d_l_f.x - sin(DEG2RAD(degree_z)) * objects1[i].d_l_f.y;
                    bottom_corners[1].y = sin(DEG2RAD(degree_z)) * objects1[i].d_l_f.x + cos(DEG2RAD(degree_z)) * objects1[i].d_l_f.y;
                    bottom_corners[1].z = objects1[i].d_l_f.z;

                    // down-right-back
                    bottom_corners[2].x = cos(DEG2RAD(degree_z)) * objects1[i].d_r_b.x - sin(DEG2RAD(degree_z)) * objects1[i].d_r_b.y;
                    bottom_corners[2].y = sin(DEG2RAD(degree_z)) * objects1[i].d_r_b.x + cos(DEG2RAD(degree_z)) * objects1[i].d_r_b.y;
                    bottom_corners[2].z = objects1[i].d_r_b.z;

                    // down-right-front
                    bottom_corners[3].x = cos(DEG2RAD(degree_z)) * objects1[i].d_r_f.x - sin(DEG2RAD(degree_z)) * objects1[i].d_r_f.y;
                    bottom_corners[3].y = sin(DEG2RAD(degree_z)) * objects1[i].d_r_f.x + cos(DEG2RAD(degree_z)) * objects1[i].d_r_f.y;
                    bottom_corners[3].z = objects1[i].d_r_f.z;

                    // up-left-back
                    top_corners[0].x = cos(DEG2RAD(degree_z)) * objects1[i].u_l_b.x - sin(DEG2RAD(degree_z)) * objects1[i].u_l_b.y;
                    top_corners[0].y = sin(DEG2RAD(degree_z)) * objects1[i].u_l_b.x + cos(DEG2RAD(degree_z)) * objects1[i].u_l_b.y;
                    top_corners[0].z = objects1[i].u_l_b.z;

                    // up-left-front
                    top_corners[1].x = cos(DEG2RAD(degree_z)) * objects1[i].u_l_f.x - sin(DEG2RAD(degree_z)) * objects1[i].u_l_f.y;
                    top_corners[1].y = sin(DEG2RAD(degree_z)) * objects1[i].u_l_f.x + cos(DEG2RAD(degree_z)) * objects1[i].u_l_f.y;
                    top_corners[1].z = objects1[i].u_l_f.z;

                    // up-right-back
                    top_corners[2].x = cos(DEG2RAD(degree_z)) * objects1[i].u_r_b.x - sin(DEG2RAD(degree_z)) * objects1[i].u_r_b.y;
                    top_corners[2].y = sin(DEG2RAD(degree_z)) * objects1[i].u_r_b.x + cos(DEG2RAD(degree_z)) * objects1[i].u_r_b.y;
                    top_corners[2].z = objects1[i].u_r_b.z;

                    // up-right-front
                    top_corners[3].x = cos(DEG2RAD(degree_z)) * objects1[i].u_r_f.x - sin(DEG2RAD(degree_z)) * objects1[i].u_r_f.y;
                    top_corners[3].y = sin(DEG2RAD(degree_z)) * objects1[i].u_r_f.x + cos(DEG2RAD(degree_z)) * objects1[i].u_r_f.y;
                    top_corners[3].z = objects1[i].u_r_f.z;

                    // down-left-back
                    x_mls_1 = objects2[j].d_l_b.x;
                    y_mls_1 = objects2[j].d_l_b.y;
                    z_mls_1 = objects2[j].d_l_b.z;
                    for (int i = 0; i < 4; i++)
                    {
                        dx = x_mls_1 - bottom_corners[i].x;
                        dy = y_mls_1 - bottom_corners[i].y;
                        dz = z_mls_1 - bottom_corners[i].z;

                        if (dx < -1.0f * x_max || dx > x_max || dy < -1.0f * y_max || dy > y_max || dz < -1.0f * z_max || dz > z_max)
                        {
                            to_be_continued = true;
                            break;
                        }

                        dx_res = (int)((dx + x_max) / planar_resolution);
                        dy_res = (int)((dy + y_max) / planar_resolution);
                        dz_res = (int)((dz + z_max) / z_resolution);

                        transformsAcc[rot_][dx_res][dy_res][dz_res].i += vote_weigth;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].x += dx;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].y += dy;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].z += dz;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].rotz = degree_z;
                    }
                    if (to_be_continued)
                    {
                        to_be_continued = false;
                        continue;
                    }
                    x_mls_2 = objects2[j].d_l_f.x;
                    y_mls_2 = objects2[j].d_l_f.y;
                    z_mls_2 = objects2[j].d_l_f.z;
                    for (int i = 0; i < 4; i++)
                    {
                        dx = x_mls_2 - bottom_corners[i].x;
                        dy = y_mls_2 - bottom_corners[i].y;
                        dz = z_mls_2 - bottom_corners[i].z;

                        if (dx < -1.0f * x_max || dx > x_max || dy < -1.0f * y_max || dy > y_max || dz < -1.0f * z_max || dz > z_max)
                        {
                            to_be_continued = true;
                            break;
                        }

                        dx_res = (int)((dx + x_max) / planar_resolution);
                        dy_res = (int)((dy + y_max) / planar_resolution);
                        dz_res = (int)((dz + z_max) / z_resolution);

                        transformsAcc[rot_][dx_res][dy_res][dz_res].i += vote_weigth;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].x += dx;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].y += dy;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].z += dz;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].rotz = degree_z;
                    }
                    if (to_be_continued)
                    {
                        to_be_continued = false;
                        continue;
                    }
                    x_mls_3 = objects2[j].d_r_b.x;
                    y_mls_3 = objects2[j].d_r_b.y;
                    z_mls_3 = objects2[j].d_r_b.z;
                    for (int i = 0; i < 4; i++)
                    {
                        dx = x_mls_3 - bottom_corners[i].x;
                        dy = y_mls_3 - bottom_corners[i].y;
                        dz = z_mls_3 - bottom_corners[i].z;

                        if (dx < -1.0f * x_max || dx > x_max || dy < -1.0f * y_max || dy > y_max || dz < -1.0f * z_max || dz > z_max)
                        {
                            to_be_continued = true;
                            break;
                        }

                        dx_res = (int)((dx + x_max) / planar_resolution);
                        dy_res = (int)((dy + y_max) / planar_resolution);
                        dz_res = (int)((dz + z_max) / z_resolution);

                        transformsAcc[rot_][dx_res][dy_res][dz_res].i += vote_weigth;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].x += dx;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].y += dy;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].z += dz;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].rotz = degree_z;
                    }
                    if (to_be_continued)
                    {
                        to_be_continued = false;
                        continue;
                    }
                    x_mls_4 = objects2[j].d_r_f.x;
                    y_mls_4 = objects2[j].d_r_f.y;
                    z_mls_4 = objects2[j].d_r_f.z;
                    for (int i = 0; i < 4; i++)
                    {
                        dx = x_mls_4 - top_corners[i].x;
                        dy = y_mls_4 - top_corners[i].y;
                        dz = z_mls_4 - top_corners[i].z;

                        if (dx < -1.0f * x_max || dx > x_max || dy < -1.0f * y_max || dy > y_max || dz < -1.0f * z_max || dz > z_max)
                        {
                            to_be_continued = true;
                            break;
                        }

                        dx_res = (int)((dx + x_max) / planar_resolution);
                        dy_res = (int)((dy + y_max) / planar_resolution);
                        dz_res = (int)((dz + z_max) / z_resolution);

                        transformsAcc[rot_][dx_res][dy_res][dz_res].i += vote_weigth;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].x += dx;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].y += dy;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].z += dz;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].rotz = degree_z;
                    }
                    if (to_be_continued)
                    {
                        to_be_continued = false;
                        continue;
                    }
                    x_mls_5 = objects2[j].u_l_b.x;
                    y_mls_5 = objects2[j].u_l_b.y;
                    z_mls_5 = objects2[j].u_l_b.z;
                    for (int i = 0; i < 4; i++)
                    {
                        dx = x_mls_5 - top_corners[i].x;
                        dy = y_mls_5 - top_corners[i].y;
                        dz = z_mls_5 - top_corners[i].z;

                        if (dx < -1.0f * x_max || dx > x_max || dy < -1.0f * y_max || dy > y_max || dz < -1.0f * z_max || dz > z_max)
                        {
                            to_be_continued = true;
                            break;
                        }

                        dx_res = (int)((dx + x_max) / planar_resolution);
                        dy_res = (int)((dy + y_max) / planar_resolution);
                        dz_res = (int)((dz + z_max) / z_resolution);

                        transformsAcc[rot_][dx_res][dy_res][dz_res].i += vote_weigth;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].x += dx;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].y += dy;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].z += dz;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].rotz = degree_z;
                    }
                    if (to_be_continued)
                    {
                        to_be_continued = false;
                        continue;
                    }
                    x_mls_6 = objects2[j].u_l_f.x;
                    y_mls_6 = objects2[j].u_l_f.y;
                    z_mls_6 = objects2[j].u_l_f.z;
                    for (int i = 0; i < 4; i++)
                    {
                        dx = x_mls_6 - top_corners[i].x;
                        dy = y_mls_6 - top_corners[i].y;
                        dz = z_mls_6 - top_corners[i].z;

                        if (dx < -1.0f * x_max || dx > x_max || dy < -1.0f * y_max || dy > y_max || dz < -1.0f * z_max || dz > z_max)
                        {
                            to_be_continued = true;
                            break;
                        }

                        dx_res = (int)((dx + x_max) / planar_resolution);
                        dy_res = (int)((dy + y_max) / planar_resolution);
                        dz_res = (int)((dz + z_max) / z_resolution);

                        transformsAcc[rot_][dx_res][dy_res][dz_res].i += vote_weigth;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].x += dx;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].y += dy;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].z += dz;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].rotz = degree_z;
                    }
                    if (to_be_continued)
                    {
                        to_be_continued = false;
                        continue;
                    }
                    x_mls_7 = objects2[j].u_r_b.x;
                    y_mls_7 = objects2[j].u_r_b.y;
                    z_mls_7 = objects2[j].u_r_b.z;
                    for (int i = 0; i < 4; i++)
                    {
                        dx = x_mls_7 - top_corners[i].x;
                        dy = y_mls_7 - top_corners[i].y;
                        dz = z_mls_7 - top_corners[i].z;

                        if (dx < -1.0f * x_max || dx > x_max || dy < -1.0f * y_max || dy > y_max || dz < -1.0f * z_max || dz > z_max)
                        {
                            to_be_continued = true;
                            break;
                        }

                        dx_res = (int)((dx + x_max) / planar_resolution);
                        dy_res = (int)((dy + y_max) / planar_resolution);
                        dz_res = (int)((dz + z_max) / z_resolution);

                        transformsAcc[rot_][dx_res][dy_res][dz_res].i += vote_weigth;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].x += dx;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].y += dy;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].z += dz;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].rotz = degree_z;
                    }
                    if (to_be_continued)
                    {
                        to_be_continued = false;
                        continue;
                    }
                    x_mls_8 = objects2[j].u_r_f.x;
                    y_mls_8 = objects2[j].u_r_f.y;
                    z_mls_8 = objects2[j].u_r_f.z;
                    for (int i = 0; i < 4; i++)
                    {
                        dx = x_mls_8 - top_corners[i].x;
                        dy = y_mls_8 - top_corners[i].y;
                        dz = z_mls_8 - top_corners[i].z;

                        if (dx < -1.0f * x_max || dx > x_max || dy < -1.0f * y_max || dy > y_max || dz < -1.0f * z_max || dz > z_max)
                        {
                            to_be_continued = true;
                            break;
                        }

                        dx_res = (int)((dx + x_max) / planar_resolution);
                        dy_res = (int)((dy + y_max) / planar_resolution);
                        dz_res = (int)((dz + z_max) / z_resolution);

                        transformsAcc[rot_][dx_res][dy_res][dz_res].i += vote_weigth;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].x += dx;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].y += dy;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].z += dz;
                        transformsAcc[rot_][dx_res][dy_res][dz_res].rotz = degree_z;
                    }
                    if (to_be_continued)
                    {
                        to_be_continued = false;
                        continue;
                    }
                }
            }
        }
    }

    // Find maximum value
    float max_i = 0.0;
    for (int i = 0; i < rot_dim_z; ++i)
    {
        for (int l = 0; l < x_dim; ++l)
        {
            for (int m = 0; m < y_dim; ++m)
            {
                for (int n = 0; n < z_dim; ++n)
                {
                    if (transformsAcc[i][l][m][n].i > max_i)
                    {
                        max_i = transformsAcc[i][l][m][n].i;
                        idx_x = l;
                        idx_y = m;
                        idx_z = n;
                        idx_rot_z = i;
                    }
                }
            }
        }
    }
    float threshold = 0.0f;
    
    // Averaging based on the max votes (enables going under resolution size)
    imax = transformsAcc[idx_rot_z][idx_x][idx_y][idx_z].x / (float)max_i;
    jmax = transformsAcc[idx_rot_z][idx_x][idx_y][idx_z].y / (float)max_i;
    kmax = transformsAcc[idx_rot_z][idx_x][idx_y][idx_z].z / (float)max_i;
    rotmax_z = transformsAcc[idx_rot_z][idx_x][idx_y][idx_z].rotz;
    if (max_i >= threshold)
    {
        std::cout << "\033[;32mMaximum: " << maximum << " Current votes: " << max_i << " Threshold: " << threshold <<"\033[0m"<< std::endl;
        //Ez a lekérdezhető mező
        goodMatch = true;
        post_transform(0, 0) = cos(DEG2RAD(rotmax_z));
        post_transform(0, 1) = -sin(DEG2RAD(rotmax_z));
        post_transform(1, 0) = sin(DEG2RAD(rotmax_z));
        post_transform(1, 1) = cos(DEG2RAD(rotmax_z));

        post_transform(0, 3) = imax;
        post_transform(1, 3) = jmax;
        post_transform(2, 3) = kmax;
    }else{
        std::cout << "\033[;31mMaximum: " << maximum << " Current votes: " << max_i << " Threshold: " << threshold << "\033[0m" << std::endl;
    }
  
    //std::cout << "Maximum votes: " << max_i << std::endl;

    std::cout << "Rotation angle: " << rotmax_z << std::endl;
    std::cout << "x= " << imax << std::endl;
    std::cout << "y= " << jmax << std::endl;
    std::cout << "z= " << kmax << std::endl;

    for (int i = 0; i < rot_dim_z; ++i)
    {
        for (int l = 0; l < x_dim; ++l)
        {
            for (int m = 0; m < y_dim; ++m)
            {
                delete[] transformsAcc[i][l][m];
            }
            delete[] transformsAcc[i][l];
        }
        delete[] transformsAcc[i];
    }
    delete[] transformsAcc;

    return post_transform;
}
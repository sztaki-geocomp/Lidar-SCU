#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>


#define PI 3.14159265
typedef pcl::PointXYZRGB PointTypeFull;
typedef bool myFunction(const PointTypeFull &point_a, const PointTypeFull &point_b, float squared_distance);
class boundingboxEporter
{
public:
    enum class_type
    {
        wall,
        columnn,
        plant
    };
    struct bbox
    {
        pcl::PointXYZRGB points[8]; // bottom 0-3, top 0-3
        pcl::PointXYZRGB center;
        class_type object_class;
        int number_of_indeces = -1;
        float b_angle = 0.0;
        float volume = 0.0;
        float width = 0.0;  // x min max
        float length = 0.0; // y min max
        float height = 0.0; // z min max
        std::vector<int> overlap_with;
        int overlap_count = 0;
    };
    std::string destination = "";
    std::string destination_for_objects = "";
    std::string destination_for_txt = "";
    int file_name_counter = 0;
    std::ofstream file;
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bounding;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster;
    pcl::IndicesClustersPtr clusters_conditional;
    std::vector<bbox> bboxes;

    boundingboxEporter(std::string _destination_for_objects, std::string _destination, std::string _destination_for_txt, std::string source) : cloud(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                                                                                                               cloud_bounding(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>()),clusters_conditional(new pcl::IndicesClusters)
    {
        destination_for_objects = _destination_for_objects;
        destination = _destination;
        destination_for_txt = _destination_for_txt;
        std::cout << "====================================================================================" << std::endl;
        std::cout << "OBJECT CLUSTERING" << std::endl;
        std::cout << "LOADING CLOUD FROM: " << source << std::endl;
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(source, *cloud) == -1)
        {
            PCL_ERROR("COULD NOT READ FILE\n");
            exit(-1);
        }
        std::cout << "LOADED CLOUD SIZE: " << cloud->size() << std::endl;
        pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, *cloud_segmented);
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cloud(){
        return cloud;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_segmented_cloud(){
        return cloud_segmented;
    }
    void downsampling(float leaf_size)
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*cloud);
        std::cout << "DOWNSAMPLED CLOUD SIZE: " << cloud->size() << std::endl;
    }
    void to_pcd()
    {
        try
        {
            pcl::io::savePCDFileASCII(destination, *cloud);
            std::cout << "==============================================================" << std::endl;
            std::cerr << "SAVED FILE'S SIZE: " << cloud->size() << " SAVED INTO:" << destination << std::endl;
            std::cout << "==============================================================" << std::endl;
        }
        catch (pcl::IOException e1)
        {
            std::cout << "==============================================================" << std::endl;
            std::cout << "IOException CATCHED" << std::endl;
            std::cout << "==============================================================" << std::endl;
        }
        //cloud->erase(cloud->begin(), cloud->end());
    }
    void cluster_conditional(myFunction &enforceIntensitySimilarity)
    {
        std::cout << "RUNNING CONDITIONAL CLUSTERING..." << std::endl;
        float tolerance = 0.8;
        int min_cluster = 5;
        int max_cluster = 1000000;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud);

        pcl::ConditionalEuclideanClustering<PointTypeFull> cec(true);
        pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);

        cec.setConditionFunction(&enforceIntensitySimilarity);
        cec.setClusterTolerance(tolerance);
        cec.setMinClusterSize(min_cluster);
        cec.setMaxClusterSize(max_cluster);
        cec.setInputCloud(cloud);
        cec.segment(*clusters_conditional);
    }
    pcl::PointXYZRGB get_center(bbox boundingbox)
    {
        pcl::PointXYZRGB center;
        center.x = ((boundingbox.points[5].x + boundingbox.points[2].x) / 2.0);
        center.y = ((boundingbox.points[5].y + boundingbox.points[2].y) / 2.0);
        center.z = ((boundingbox.points[5].z + boundingbox.points[2].z) / 2.0);
        return center;
    }
    class_type get_object_type(bbox boundingbox)
    {
        class_type object_type;
        float wall_length, wall_width = 3.0;
        float wall_height = 1.5;
        float column_length = 4.0;
        float column_width = 4.0;

        if (boundingbox.volume > 60.0 || (boundingbox.width > column_width || boundingbox.length > column_length))
        {
            object_type = wall;
        }
        else
        {
            object_type = columnn;
        }
        return object_type;
    }

    void get_bboxes()
    {
        std::cout << "SEARCHING FOR CORNERS..." << std::endl;
        pcl::PointXYZRGB min_pt, max_pt;
        pcl::PointXYZRGB s_min_pt, s_max_pt;
        float volume;
        int oszto = 36;
        float theta = M_PI / oszto;
        int counter = 0;
        pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
        std::vector<float> moment_of_inertia;
        std::vector<float> eccentricity;
        pcl::PointXYZRGB min_point_AABB;
        pcl::PointXYZRGB max_point_AABB;
        pcl::PointXYZRGB min_point_OBB;
        pcl::PointXYZRGB max_point_OBB;
        pcl::PointXYZRGB position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;
        float angle = 0.0f;
        int minx, miny, minz, maxx, maxy, maxz = -1;
        int indexes[6];
        std::vector<pcl::PointIndices> clusters = *clusters_conditional;
        for (const auto &cluster : clusters)
        {
            minx, miny, minz, maxx, maxy, maxz = -1;
            volume = 10000.0;
            min_pt.x = 1000000;
            min_pt.y = 1000000;
            min_pt.z = 1000000;
            max_pt.x = -1000000;
            max_pt.y = -1000000;
            max_pt.z = -1000000;
            cloud_out->erase(cloud_out->begin(), cloud_out->end());
            cloud_bounding->erase(cloud_bounding->begin(), cloud_bounding->end());
            for (const auto &j : cluster.indices)
            {
                cloud_out->push_back((*cloud)[j]);
            }

            bbox boundingbox;
            boundingbox.number_of_indeces = counter;
            counter++;
            pcl::PointXYZRGB minPt, maxPt;
            pcl::getMinMax3D(*cloud_out, minPt, maxPt);
            pcl::PointXYZ center;
            center.x = std::abs(minPt.x + maxPt.x) / 2;
            center.y = std::abs(minPt.y + maxPt.y) / 2;
            center.z = std::abs(minPt.z + maxPt.z) / 2;
            Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
            transform_1(0, 3) = -center.x;
            transform_1(1, 3) = -center.y;
            pcl::transformPointCloud(*cloud_out, *cloud_out, transform_1);
            float tmp_volume = 0.0f;
            for (int rotate = 1; rotate <= oszto; rotate++)
            {
                min_pt.x = 1000000;
                min_pt.y = 1000000;
                min_pt.z = 1000000;
                max_pt.x = -1000000;
                max_pt.y = -1000000;
                max_pt.z = -1000000;
                Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

                transform_2(0, 0) = std::cos(theta);
                transform_2(0, 1) = -sin(theta);
                transform_2(1, 0) = sin(theta);
                transform_2(1, 1) = std::cos(theta);
                pcl::transformPointCloud(*cloud_out, *cloud_out, transform_2);

                for (int idx = 0; idx < cloud_out->size(); idx++)
                {
                    if ((*cloud_out)[idx].x > max_pt.x)
                    {
                        max_pt.x = (*cloud_out)[idx].x;
                        maxx = idx;
                    }
                    if ((*cloud_out)[idx].y > max_pt.y)
                    {
                        max_pt.y = (*cloud_out)[idx].y;
                        maxy = idx;
                    }
                    if ((*cloud_out)[idx].z > max_pt.z)
                    {
                        max_pt.z = (*cloud_out)[idx].z;
                        maxz = idx;
                    }
                    if ((*cloud_out)[idx].x < min_pt.x)
                    {
                        min_pt.x = (*cloud_out)[idx].x;
                        minx = idx;
                    }
                    if ((*cloud_out)[idx].y < min_pt.y)
                    {
                        min_pt.y = (*cloud_out)[idx].y;
                        miny = idx;
                    }
                    if ((*cloud_out)[idx].z < min_pt.z)
                    {
                        min_pt.z = (*cloud_out)[idx].z;
                        minz = idx;
                    }
                }

                float tmp_x = std::abs(max_pt.x - min_pt.x);
                float tmp_y = std::abs(max_pt.y - min_pt.y);
                float tmp_z = std::abs(max_pt.z - min_pt.z);
                tmp_volume = tmp_x * tmp_y * tmp_z;
                if (tmp_volume < volume)
                {
                    volume = tmp_volume;
                    angle = theta * (rotate);
                    minPt.x = min_pt.x;
                    minPt.y = min_pt.y;
                    minPt.z = min_pt.z;
                    maxPt.x = max_pt.x;
                    maxPt.y = max_pt.y;
                    maxPt.z = max_pt.z;
                }
            }

            pcl::PointXYZRGB point1;
            pcl::PointXYZRGB point2;
            pcl::PointXYZRGB point3;
            pcl::PointXYZRGB point4;
            pcl::PointXYZRGB point5;
            pcl::PointXYZRGB point6;
            pcl::PointXYZRGB point7;
            pcl::PointXYZRGB point8;
            point1.x = maxPt.x;
            point1.y = minPt.y;
            point1.z = minPt.z;
            point2.x = maxPt.x;
            point2.y = maxPt.y;
            point2.z = minPt.z;
            point3.x = minPt.x;
            point3.y = minPt.y;
            point3.z = minPt.z;
            point4.x = minPt.x;
            point4.y = maxPt.y;
            point4.z = minPt.z;
            point5.x = maxPt.x;
            point5.y = minPt.y;
            point5.z = maxPt.z;
            point6.x = maxPt.x;
            point6.y = maxPt.y;
            point6.z = maxPt.z;
            point7.x = minPt.x;
            point7.y = minPt.y;
            point7.z = maxPt.z;
            point8.x = minPt.x;
            point8.y = maxPt.y;
            point8.z = maxPt.z;
            cloud_bounding->push_back(point1);
            cloud_bounding->push_back(point2);
            cloud_bounding->push_back(point3);
            cloud_bounding->push_back(point4);
            cloud_bounding->push_back(point5);
            cloud_bounding->push_back(point6);
            cloud_bounding->push_back(point7);
            cloud_bounding->push_back(point8);
            Eigen::Matrix4f rotate_back = Eigen::Matrix4f::Identity();
            // The angle of rotation in radians
            rotate_back(0, 0) = std::cos(-angle);
            rotate_back(0, 1) = -sin(-angle);
            rotate_back(1, 0) = sin(-angle);
            rotate_back(1, 1) = std::cos(-angle);
            pcl::transformPointCloud(*cloud_bounding, *cloud_bounding, rotate_back);
            Eigen::Matrix4f move_back = Eigen::Matrix4f::Identity();
            move_back(0, 3) = center.x;
            move_back(1, 3) = center.y;
            pcl::transformPointCloud(*cloud_bounding, *cloud_bounding, move_back);
            boundingbox.points[0] = (*cloud_bounding)[0];
            boundingbox.points[1] = (*cloud_bounding)[1];
            boundingbox.points[2] = (*cloud_bounding)[2];
            boundingbox.points[3] = (*cloud_bounding)[3];
            boundingbox.points[4] = (*cloud_bounding)[4];
            boundingbox.points[5] = (*cloud_bounding)[5];
            boundingbox.points[6] = (*cloud_bounding)[6];
            boundingbox.points[7] = (*cloud_bounding)[7];
            boundingbox.center = get_center(boundingbox);
            boundingbox.b_angle = angle;
            boundingbox.width = std::abs(sqrt(pow(boundingbox.points[0].x - boundingbox.points[2].x, 2.0) + pow(boundingbox.points[0].y - boundingbox.points[2].y, 2.0)));
            boundingbox.length = std::abs(sqrt(pow(boundingbox.points[0].x - boundingbox.points[1].x, 2.0) + pow(boundingbox.points[0].y - boundingbox.points[1].y, 2.0)));
            boundingbox.height = std::abs(boundingbox.points[0].z - boundingbox.points[4].z);
            boundingbox.volume = volume;
            boundingbox.object_class = get_object_type(boundingbox);
            bboxes.push_back(boundingbox);
        }
    }
    std::string get_corners_to_string(std::vector<bbox> bboxes, int j)
    {
        std::string tmp;
        int counter = 0;
        for (int i = 0; i < 8; i++)
        {
            tmp += ";";
            tmp += std::to_string(bboxes[j].points[i].x);
            tmp += ";";
            tmp += std::to_string(bboxes[j].points[i].y);
            tmp += ";";
            tmp += std::to_string(bboxes[j].points[i].z);
            counter++;
        }
        return tmp;
    }
    void save_specific_object(bool save_modified_cloud, bool save_all_object_as_cloud)
    {
        std::cout << "EXPORTING CORNERS..." << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::ofstream myfile;
        myfile.open(destination_for_txt + ".txt");
        for (int i = 0; i < bboxes.size(); i++)
        {
            pcl::PointXYZRGB center = bboxes[i].center;
            center.r = 0;
            center.g = 255;
            center.b = 0;
            if (save_modified_cloud)
            {
                std::vector<pcl::PointIndices> clusters = *clusters_conditional;
                for (const auto &j : clusters[bboxes[i].number_of_indeces].indices)
                {
                    if (bboxes[i].object_class == wall)
                    {
                        (*cloud)[j].r = 255;
                        (*cloud)[j].g = 140;
                        (*cloud)[j].b = 0;
                    }
                    else
                    {
                        (*cloud)[j].r = 255;
                        (*cloud)[j].g = 0;
                        (*cloud)[j].b = 0;
                    }
                    cloud_object->push_back(((*cloud)[j]));
                }
                for (int j = 0; j < 8; j++)
                {
                    pcl::PointXYZRGB tmp;
                    tmp.x = bboxes[i].points[j].x;
                    tmp.y = bboxes[i].points[j].y;
                    tmp.z = bboxes[i].points[j].z;
                    tmp.r = 0;
                    tmp.g = 255;
                    tmp.b = 0;
                    cloud_object->push_back(tmp);
                }
            }
            std::string classification = "";
            if (bboxes[i].object_class == wall)
            {
                classification = "wall";
            }
            else if (bboxes[i].object_class == columnn)
            {
                classification = "columnn";
            }
            else
            {
                classification = "other";
            }
            std::string object = "";
            object = classification + ";" + std::to_string(bboxes[i].center.x) +
                     ";" + std::to_string(bboxes[i].center.y) + ";" + std::to_string(bboxes[i].center.z) + get_corners_to_string(bboxes, i) + ";" + std::to_string(bboxes[i].height) + ";" +
                     std::to_string(bboxes[i].width) + ";" + std::to_string(bboxes[i].length) + ";" + std::to_string(bboxes[i].volume) + ";" + std::to_string(bboxes[i].b_angle);

            myfile << object << "\n";
            if (save_all_object_as_cloud)
            {
                std::string tmp = destination_for_objects + std::to_string(i) + ".pcd";
                cloud_object->push_back(center);
                pcl::io::savePCDFileASCII(tmp, *cloud_object);
                std::cout << "OBJECT CLOUD SIZE: " << cloud_object->size() << std::endl;
                cloud_object->erase(cloud_object->begin(), cloud_object->end());
            }
        }
        myfile.close();
    }
};
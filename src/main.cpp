#define _CRT_SECURE_NO_WARNINGS
#include "processMLS.cpp"
#include "sceneInterpreter.h"
#include "Registration.h"
#include "rayTracer.hpp"
#include "MRFWrapper.h"
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGB PointTypeFull;
typedef bool myFunction(const PointTypeFull &point_a, const PointTypeFull &point_b, float squared_distance);

std::vector<MyObject> objects_velo;
float logistic(float &v, float a=1.0f,float b = 0.0f, float L = 1.0f) {
    return L / (1.0f + exp(a*v-b));
  };

float gaussian2D(float x, float y, float mx, float my, float sx, float sy)
{
    float ax = (x - mx)*(x - mx) / (2*sx*sx);
    float ay = (y - my)*(y - my) / (2*sy*sy);

    return 1.0f / (2*3.1415*sx*sy) * std::exp(-(ax + ay));
}
// Function to restrict the difference between two points during the clustering process.
// Increasing delta results in a more permissive condition.
bool enforceIntensitySimilarity(const PointTypeFull &point_a, const PointTypeFull &point_b, float squared_distance)
{
	float delta = 30.0f;
	if (std::abs(point_a.r - point_b.r) < delta && std::abs(point_a.b - point_b.b) < delta && std::abs(point_a.b - point_b.b) < delta)
		return (true);
	else
		return (false);
}
// Point cloud viewer fuctions
void viewCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud, std::string wName)
{
	pcl::visualization::CloudViewer viewer(wName);
	viewer.showCloud(ptrCloud);
	while (!viewer.wasStopped())
	{
	}
}
void viewCloudPair(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud2, std::string wName)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	for (auto p:*ptrCloud1){
		p.r = 255;
		p.g = 0;
		p.b = 0;
		resultCloud->points.push_back(p);
	}
	for (auto p:*ptrCloud2){
		p.r = 0;
		p.g = 0;
		p.b = 255;
		resultCloud->points.push_back(p);
	}
	pcl::visualization::CloudViewer viewer(wName);
	viewer.showCloud(resultCloud);
	while (!viewer.wasStopped())
	{
	}
}
// All the required processes for exporting minimum volume bounding box corners, types and orientation are listed in this function.
void MLSBoundingBoxes(boundingboxEporter *processMLSCloud)
{
	std::cout << "WORKING ON EXPORTING MINIMUME VOLUME BOUNDINGBOX CONER COORDINATES, OBJECT TYPE AND ORIENTATION..." << std::endl;
	// Downsampling the incoming cloud with pcl functions to save time.
	processMLSCloud->downsampling(0.1);
	// Using conditional clustering to cluster the points. The condition restricts the difference between two point's colours.
	processMLSCloud->cluster_conditional(enforceIntensitySimilarity);
	// The minimum volume bounding boxes, corners and other parameters are calculated in this funtion.
	processMLSCloud->get_bboxes();
	// Exporting the data in unique txt format.
	// Set the first function parameter to true, in case the modified cloud is needed.
	// Set the second function parameter to true, in case all clustered object is needed in individual clouds.
	processMLSCloud->save_specific_object(true, false);
	// The modified cloud is saved for further verification and correctness testing.
	processMLSCloud->to_pcd();
	// To view point cloud data. This part is not necessary to execute, only included because of the representation.
	// Maybe zooming out is required.
	std::cout<<"VISUALISATION STARTED";
	//MIN MAX CALCULATION
	// pcl::PointXYZRGB minPt, maxPt;
  	// pcl::getMinMax3D (*processMLSCloud->get_cloud(), minPt, maxPt);
	// Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
	// //TRANSFORMING MLS CLOUD TO ORIGIN.
	// float tran_x = -(maxPt.x-((maxPt.x - minPt.x) / 2));
	// float tran_y = -(maxPt.y-((maxPt.y - minPt.y) / 2));
	// float tran_z = -(maxPt.z-((maxPt.z - minPt.z) / 2));
	// trans(0, 3) = tran_x;
	// trans(1, 3) = tran_y;
	// trans(2, 3) = tran_z;
	// pcl::transformPointCloud(*processMLSCloud->get_cloud(), *processMLSCloud->get_cloud(), trans);
	viewCloud(processMLSCloud->get_cloud(), "Detected MLS landmarks");

}
// All the required processes for extracting ground plane and detecting pillar like objects are listed in this function.
void VelodyneObjectDetection(SceneIP2D *scene2D, std::string save_modified_velodyne)
{
	std::cout << "VELODYNE CLOUD OBJECT DETECTION..." << std::endl;
	// Setting up the scene2d object with desired parameters. VoxelSize has serious effect on the performance.
	scene2D->setVoxelSize(0.4);
	scene2D->setGroundSize(0.5);
	// Estimated sensor height
	scene2D->setSensorHeight(2.0);
	// Initializing the scene before executing the objectDetection
	scene2D->buildScene();
	scene2D->objectDetection();
	// This part is only included to represent and classify the result of the detection algorithm.
	// The velodyne cloud is simple modified. The pillar class is represented in the saved cloud with red colour.
	float rx, ry, rz;
	// Iterates over the results of the detection and determines whether it is pillar like or not.
	// Vector of the detected objects is decleared
	std::cout << "OBJECTS: " << scene2D->objects.size() << std::endl;
	for (auto &obj : scene2D->objects)
	{
		if (obj._points.size() > 0)
		{
			rx = obj.max.x - obj.min.x;
			ry = obj.max.y - obj.min.y;
			rz = obj.max.z - obj.min.z;
			bool b = rz > 1.0f && rz < 8.0;				 // Not too short, not too high
			bool bb = rx < 10.0 && ry < 10.0;			 // Not too long
			bool col = rz > 2.0f * rx && rz > 2.0f * ry; // Pillar-like
			if (obj._points.size() > 10 && bb && b && col)
			{
				objects_velo.push_back(obj);
				int r = 0;
				int b = 0;
				int g = 0;
				// This part is only included to represent the result. Pillar-like objects are coloured red.
				for (size_t j = 0; j < obj._points.size(); ++j)
				{
					if (col)
					{
						b = 0;
						r = 255;
					}
					else
					{
						r = 255;
					}
					obj._points.at(j)->r = r;
					obj._points.at(j)->g = g;
					obj._points.at(j)->b = b;
				}
			}
		}
	}
	// To remove ground
	// scene2D->colorSegment();
	// View the processed cloud. Maybe zooming out is required.
	std::cout<<"VISUALISATION PART!";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud<pcl::PointXYZRGBNormal>(*scene2D->model->spCloud, *ptrCloud);
	viewCloud(ptrCloud,"Detected Pillar objects");
	// Printing the results.
	std::cout << "NUMBER OF DETECTED PILLARS: " << objects_velo.size() << std::endl;
	// pcl::io::savePCDFileASCII(save_modified_velodyne, *scene2D->model->spCloud);
}
int main(int argc, char** argv)
{
	std::string inputVelodyneSamplePath; 
	std::string inputMLSSamplePath; 
	if (argc == 1){
		inputVelodyneSamplePath =  "../Data/Samples/Velo2.pcd";
		inputMLSSamplePath = "../Data/Samples/MLS2.pcd";
	}
	else if(argc == 2){
		std::cout<<"Not enough input arguments!"<<std::endl;
		return 0;
	}
	else if(argc == 3){
		inputVelodyneSamplePath = argv[1];
		inputMLSSamplePath = argv[2];
	}
	else if(argc > 3){
		std::cout<<"Too many input arguments!"<<std::endl;
		return 0;
	}


	/////////////// (OFFLINE) MLS cloud processing///////////////
	// Initializing the boundingboxEporter type object for processing the MLS cloud.
	// Meaning of the parameters respectively: destination for saved object clouds, destination for segmented cloud,  destination for the exported bounding box characteristics, source to import the mls cloud.
	boundingboxEporter *processMLSCloud = new boundingboxEporter("../Data/Output/", "../Data/Output/segmentedMLS.pcd", "../Data/Output/MLS_bbox", inputMLSSamplePath);
	// This function combines the required operations of the MLS cloud processing.
	MLSBoundingBoxes(processMLSCloud);

	///////////////Velodyne cloud processing///////////////
	// Initializing the SceneIP type object.
	SceneIP2D *scene2D = new SceneIP2D();
	// Loading example Velodyne HDL-64e cloud sample for object detection.
	// This is the path to the selected cloud.
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(inputVelodyneSamplePath, *scene2D->model->spCloud) == -1)
	{
		PCL_ERROR("ERROR\n");
		exit(-1);
	}
	// This function combines the required operations of the Velodyne cloud processing.
	VelodyneObjectDetection(scene2D, inputVelodyneSamplePath);

	// Cross-source Velodyne-MLS Registration
	/////////////// COARSE ALIGNMENT///////////////
	std::vector<keyPoint> objects1;
    std::vector<keyPoint> objects2;
    Eigen::Matrix4f coarse_transform, fine_transform;
	std::cout<<"Velodyne keypoint extraction"<<std::endl;
    for (auto obj : objects_velo)
    {
        keyPoint k;

        k.d_l_b.x = obj.min.x;
        k.d_l_b.y = obj.max.y;
        k.d_l_b.z = obj.min.z;

        k.d_r_b.x = obj.max.x;
        k.d_r_b.y = obj.max.y;
        k.d_r_b.z = obj.min.z;

        k.d_r_f.x = obj.max.x;
        k.d_r_f.y = obj.min.y;
        k.d_r_f.z = obj.min.z;

        k.d_l_f.x = obj.min.x;
        k.d_l_f.y = obj.min.y;
        k.d_l_f.z = obj.min.z;

        k.u_l_b.x = obj.min.x;
        k.u_l_b.y = obj.max.y;
        k.u_l_b.z = obj.max.z;

        k.u_r_b.x = obj.max.x;
        k.u_r_b.y = obj.max.y;
        k.u_r_b.z = obj.max.z;

        k.u_r_f.x = obj.max.x;
        k.u_r_f.y = obj.min.y;
        k.u_r_f.z = obj.max.z;

        k.u_l_f.x = obj.min.x;
        k.u_l_f.y = obj.min.y;
        k.u_l_f.z = obj.max.z;

        k.max = obj.max;
        k.min = obj.min;
        k.V = (obj.max.x - obj.min.x) * (obj.max.y - obj.min.y) * (obj.max.z - obj.min.z);
        k.cells = obj.cells;
        k.number_of_points = obj._points.size();
        k.cluster = obj.cluster;
        k.center_point.x = (obj.min.x + obj.max.x) / 2;
        k.center_point.y = (obj.min.y + obj.max.y) / 2;
        k.center_point.z = (obj.min.z + obj.max.z) / 2;

        objects1.push_back(k);
    }
    int column = 0;
	std::cout<<"MLS keypoint extraction"<<std::endl;
    for (auto bbox : processMLSCloud->bboxes)
    {
        
        if (bbox.object_class == boundingboxEporter::class_type::columnn && bbox.height > 1.0)
        {
			column++;
            keyPoint k;
            int r = 100;
            int g = 140;
            int b = 0;
            k.d_l_b.x = bbox.points[3].x; //obj.min.x;
            k.d_l_b.y = bbox.points[3].y; //obj.max.y;
            k.d_l_b.z = bbox.points[3].z; //obj.min.z;
            k.d_l_b.r = r;
            k.d_l_b.g = g;
            k.d_l_b.b = b;

            k.d_r_b.x = bbox.points[1].x; //obj.max.x;
            k.d_r_b.y = bbox.points[1].y; //obj.max.y;
            k.d_r_b.z = bbox.points[1].z; //obj.min.z;
            k.d_r_b.r = r;                //obj.max.x;
            k.d_r_b.g = g;                //obj.max.y;
            k.d_r_b.b = b;                //obj.min.z;

            k.d_r_f.x = bbox.points[0].x; //obj.max.x;
            k.d_r_f.y = bbox.points[0].y; //obj.min.y;
            k.d_r_f.z = bbox.points[0].z; //obj.min.z;
            k.d_r_f.r = r;                //obj.max.x;
            k.d_r_f.g = g;                //obj.min.y;
            k.d_r_f.b = b;                //obj.min.z;

            k.d_l_f.x = bbox.points[2].x; //obj.min.x;
            k.d_l_f.y = bbox.points[2].y; //obj.min.y;
            k.d_l_f.z = bbox.points[2].z; //obj.min.z;
            k.d_l_f.r = r;                //obj.min.x;
            k.d_l_f.g = g;                //obj.min.y;
            k.d_l_f.b = b;                //obj.min.z;

            k.u_l_b.x = bbox.points[7].x; //obj.min.x;
            k.u_l_b.y = bbox.points[7].y; //obj.max.y;
            k.u_l_b.z = bbox.points[7].z; //obj.max.z;
            k.u_l_b.r = r;                //obj.min.x;
            k.u_l_b.g = g;                //obj.max.y;
            k.u_l_b.b = b;                //obj.max.z;

            k.u_r_b.x = bbox.points[5].x; //obj.max.x;
            k.u_r_b.y = bbox.points[5].y; //obj.max.y;
            k.u_r_b.z = bbox.points[5].z; //obj.max.z;
            k.u_r_b.r = r;                //obj.max.x;
            k.u_r_b.g = g;                //obj.max.y;
            k.u_r_b.b = b;                //obj.max.z;

            k.u_r_f.x = bbox.points[4].x; //obj.max.x;
            k.u_r_f.y = bbox.points[4].y; //obj.min.y;
            k.u_r_f.z = bbox.points[4].z; //obj.max.z;
            k.u_r_f.r = r;                //obj.max.x;
            k.u_r_f.g = g;                //obj.min.y;
            k.u_r_f.b = b;                //obj.max.z;

            k.u_l_f.x = bbox.points[6].x; //obj.min.x;
            k.u_l_f.y = bbox.points[6].y; //obj.min.y;
            k.u_l_f.z = bbox.points[6].z; //obj.max.z;
            k.u_l_f.r = r;                //obj.min.x;
            k.u_l_f.g = g;                //obj.min.y;
            k.u_l_f.b = b;                //obj.max.z;

            k.V = bbox.volume;
            k.z = bbox.height;
            k.x = bbox.width;
            k.y = bbox.length;
            k.center_point.x = bbox.center.x;
            k.center_point.y = bbox.center.y;
            k.center_point.z = bbox.center.z;
            objects2.push_back(k);
        }
    }

    std::cout << "Velodyne objects: " << objects1.size() << " MLS objects: " << objects2.size()
              << " All MLS objects in range " << processMLSCloud->bboxes.size() << " All columns in range: " << column << std::endl;

	PointCloudRegistration registration;
	coarse_transform = registration.coarseAlignment3D(objects1,objects2);
	std::cout<< coarse_transform << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud<pcl::PointXYZRGBNormal>(*scene2D->model->spCloud, *ptrCloud);
	viewCloudPair(ptrCloud,processMLSCloud->get_cloud(),"Before registration");
	pcl::transformPointCloud(*processMLSCloud->get_cloud(), *processMLSCloud->get_cloud(), coarse_transform.inverse());
	viewCloudPair(ptrCloud,processMLSCloud->get_cloud(),"After registration");

	////////////// ICP-based fine alignment ///////////////
	fine_transform = registration.fineAlignment(processMLSCloud->get_cloud(),ptrCloud);
	pcl::transformPointCloud(*processMLSCloud->get_cloud(), *processMLSCloud->get_cloud(), fine_transform);
	viewCloudPair(ptrCloud,processMLSCloud->get_cloud(),"After ICP refinement");
	
	Eigen::Matrix4f result_transform = fine_transform*coarse_transform.inverse();
	pcl::transformPointCloud(*processMLSCloud->get_segmented_cloud(), *processMLSCloud->get_segmented_cloud(), result_transform);
	
	////////////////////// CHANGE DETECTION ///////////////
	// Range image generation
	int HEIGHT = 64;
    int WIDTH = 1042;
	cv::Mat veloDstVal(HEIGHT, WIDTH, CV_32F);
    cv::Mat mlsDstVal(HEIGHT, WIDTH, CV_32F);
    cv::Mat changeDstVal(HEIGHT, WIDTH, CV_32F);
	cv::Mat vegDstVal(HEIGHT, WIDTH, CV_32F);
	cv::Mat mlsLabel(HEIGHT, WIDTH, CV_8UC1);
	// Center point of projection
	pcl::PointXYZRGB cp;
	cp.x = 0.0f;
	cp.y = 0.0f;
	cp.z = 0.0f;
	rayTracer tracer(ptrCloud, processMLSCloud->get_segmented_cloud(), cp);
    tracer.setMaxd(35.0f);
    tracer.getMinMaxAngles();
	tracer.rangeImagei3D(WIDTH, HEIGHT, veloDstVal);
    tracer.rangeImageMLS(WIDTH, HEIGHT, mlsLabel, mlsDstVal);

	cv::Mat labels = cv::Mat::zeros(HEIGHT, WIDTH, 0);
	std::vector<cv::Mat> logProbs;
	float maxdist = 0.0f;

    // Geometric distance
    for(int i = 0;i<HEIGHT;i++){
        for(int j = 0;j<WIDTH;j++){
            // No data -> no change
            if(veloDstVal.at<float>(i,j) > 90.0f || veloDstVal.at<float>(i,j) < 0.001f){
                changeDstVal.at<float>(i,j) = -15.0f;
            }
            else
            {
				std::vector<float> dsts;
                dsts.push_back(fabs(mlsDstVal.at<float>(i-1,j-1) - veloDstVal.at<float>(i,j)));
                dsts.push_back(fabs(mlsDstVal.at<float>(i  ,j-1) - veloDstVal.at<float>(i,j)));
                dsts.push_back(fabs(mlsDstVal.at<float>(i+1,j-1) - veloDstVal.at<float>(i,j)));
                dsts.push_back(fabs(mlsDstVal.at<float>(i-1,j  ) - veloDstVal.at<float>(i,j)));
                dsts.push_back(fabs(mlsDstVal.at<float>(i  ,j  ) - veloDstVal.at<float>(i,j)));
                dsts.push_back(fabs(mlsDstVal.at<float>(i+1,j  ) - veloDstVal.at<float>(i,j)));
                dsts.push_back(fabs(mlsDstVal.at<float>(i-1,j+1) - veloDstVal.at<float>(i,j)));
                dsts.push_back(fabs(mlsDstVal.at<float>(i  ,j+1) - veloDstVal.at<float>(i,j)));
                dsts.push_back(fabs(mlsDstVal.at<float>(i+1,j+1) - veloDstVal.at<float>(i,j)));
                auto res = std::min_element(dsts.begin(), dsts.end());
                changeDstVal.at<float>(i,j) = *res;
                if(changeDstVal.at<float>(i,j) > 15.0f)
                    changeDstVal.at<float>(i,j) = 15.0f;
            }
        }
    }

    // Calculating the semantic distance
    cv::Mat vegLabels;
    cv::distanceTransform(mlsLabel, vegDstVal, vegLabels, cv::DIST_L2, cv::DIST_MASK_5,cv::DIST_LABEL_PIXEL);
    std::vector<cv::Vec2i> label_to_index;
    //Convert label to indexes
    label_to_index.push_back(cv::Vec2i(-1,-1));
    for (int row = 0; row < mlsLabel.rows; ++row)
        for (int col = 0; col < mlsLabel.cols; ++col)
            if(mlsLabel.at<uchar>(row,col)==0){
                label_to_index.push_back(cv::Vec2i(row,col));
			}

    // MRF Fitness functions

    // Multi-class
    // BG0 = Static background
    // FG = Foreground change
    // VG = Vegetation change

    cv::Mat F_BG = cv::Mat::zeros(HEIGHT, WIDTH, 5);
    cv::Mat F_FG = cv::Mat::zeros(HEIGHT, WIDTH, 5);
	cv::Mat F_VG = cv::Mat::zeros(HEIGHT, WIDTH, 5);
    
    
    // Tunable parameters
    float k = 2.0f;
    float d0 = 2.0f;
    float L = 0.01f;
    float md = 0.0f;
    float mv = 0.0f;
    float sd = 1.4f;
    float sv = 2.5f;

    for(int i = 0;i<HEIGHT;i++){
        for(int j = 0;j<WIDTH;j++){
            // d (s,s)
            float geom_dist = changeDstVal.at<float>(i,j);
            // delta (s)
			float semantic_dist = vegDstVal.at<float>(i,j);
			// Nearest veg. pixel (s')
            cv::Vec2i idx = label_to_index[vegLabels.at<int32_t>(i,j)];
            // Distance from nearest vegetation point - d(s,s')
            float dst = fabs(veloDstVal.at<float>(i,j) - mlsDstVal.at<float>(idx[0],idx[1]));
			// Limit too high values
            if(dst > 15.0f) dst = 15.0f;

            // Energy-members
			F_FG.at<float>(i,j) = -log(logistic(geom_dist, k, d0,L));
			F_BG.at<float>(i,j) = -log(L - logistic(geom_dist, k, d0,L));
            F_VG.at<float>(i,j) = -log(gaussian2D(dst, semantic_dist, md, mv, sd, sv));              
        }
    }

	logProbs.push_back(F_BG);
	logProbs.push_back(F_FG);
	logProbs.push_back(F_VG);
	MyMRF::segmentImage(logProbs, labels);

	cv::Mat maskImg(HEIGHT, WIDTH, CV_8UC1);
	cv::Mat maskMRF(HEIGHT, WIDTH, CV_8UC1);
    for(int i = 0;i<HEIGHT;i++){
        for(int j = 0;j<WIDTH;j++){
			maskMRF.at<uchar>(i,j) = labels.at<uchar>(i, j);
		}
	}
	for(int i = 0;i<HEIGHT;i++){
        for(int j = 0;j<WIDTH;j++){
			maskImg.at<uchar>(i,j) = 0;
            if(maskMRF.at<uchar>(i,j) == 1)
                maskImg.at<uchar>(i,j) = 0;
            else if(maskMRF.at<uchar>(i,j) == 0)
                maskImg.at<uchar>(i,j) = 255;
            else if(maskMRF.at<uchar>(i,j) == 2)
                maskImg.at<uchar>(i,j) = 100;  
        }
    }
	cv::imshow("CHANGE MASK with MRF", maskImg);
	cv::waitKey(0);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr simulatedCloud = tracer.backProj(maskMRF, WIDTH,  HEIGHT);
	viewCloud(simulatedCloud,"Change detection results");
	delete(processMLSCloud);
	delete(scene2D);
}
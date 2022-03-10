#ifndef RAYTRACER_H
#define RAYTRACER_H

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <boost/thread/thread.hpp>

#include <iostream>
#include <vector>
#include <numeric>
#include <cstdlib>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudT;

class rayTracer{
private:
	bool velo;
	float max_alpha;
	float min_alpha;
	float max_azimuth;
	float min_azimuth;
	float MAXD = 50.0f;

	float az_res = 0.3516f; // Default
	float ang_res = 0.3906f;
	// For backprojection
	std::array<std::vector<pcl::PointXYZRGB*>, 64*1042> _points;
	std::array<float, 64*1042> _points_min_dist;

public:
	PointT basePoint;
	pcl::PointCloud<PointT>::Ptr pVelocloud;
	pcl::PointCloud<PointT>::Ptr pMlscloud;
	pcl::PointCloud<PointT>::Ptr finalcloud; 
	vector<PointT> scloudCpyV;
	vector<PointT> scloudCpy;

	rayTracer(PointCloudT vcloud, PointCloudT mcloud, PointT point){
		pVelocloud = vcloud;
		pMlscloud = mcloud;
		finalcloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
		basePoint = point;
		velo = false;
	}
	void setMaxd(float maxd){this->MAXD = maxd;}
	float calculateAz( const float &x, const float &y );
	float getAngle( PointT p );
	void rangeImagei3D(int WIDTH, int HEIGHT, cv::Mat &dstValue);
	void interpolateDst(cv::Mat &dstValue,int WIDTH, int HEIGHT);
	void rangeImageMLS(int WIDTH, int HEIGHT,cv::Mat &labelImg, cv::Mat &dstValue);
	PointCloudT backProj(cv::Mat &MRF,int WIDTH, int HEIGHT);
	void getMinMaxAngles();
	};
#endif 
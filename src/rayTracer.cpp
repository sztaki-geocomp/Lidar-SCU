#include "rayTracer.hpp"

float rayTracer::calculateAz( const float &x, const float &y ){
		return atan2(x, y)*180.0f/M_PI + 180.0f;
	}

float rayTracer::getAngle( PointT p ){
	double dz = p.z - basePoint.z;
	double dst = sqrtf(pow(p.x-basePoint.x,2) + pow(p.y-basePoint.y,2) + pow(p.z-basePoint.z,2));
	double alpha = asin( dz / dst ) *180.0f/M_PI;
	return alpha;
}

PointCloudT rayTracer::backProj(cv::Mat &MRF, int WIDTH, int HEIGHT){
	finalcloud->points.clear();
	for(int i = 0;i<HEIGHT;i++){
        for(int j = 0;j<WIDTH;j++){
			for (auto a:_points[i*(WIDTH)+j]){
				if(MRF.at<uchar>(HEIGHT - i -1,j) == 0){
					a->r = 255;
					a->g = 0;
					a->b = 0;
				}else if(MRF.at<uchar>(HEIGHT - i -1,j) == 1){
					a->r = 0;
					a->g = 0;
					a->b = 255;
					
				}else if(MRF.at<uchar>(HEIGHT - i -1,j) == 4){
					a->r = 255;
					a->g = 153;
					a->b = 0;
					
				}else if(MRF.at<uchar>(HEIGHT - i -1,j) == 2){
					a->r = 23;
					a->g = 108;
					a->b = 0;
					
				}

				finalcloud->points.push_back(*a);
		}
        }
    }
	return finalcloud;
}

void rayTracer::getMinMaxAngles(){
	max_alpha = 0.0f;
	min_alpha = 30.0f;
	min_azimuth = 400.0f;
	max_azimuth = -400.0f;
	for(auto p : pVelocloud->points) {

		float alpha = getAngle(p);
		float azimuth = calculateAz(p.x,p.y);

		if(azimuth > max_azimuth){
			max_azimuth = azimuth;
		}
		if(azimuth < min_azimuth){
			min_azimuth = azimuth;
		}
		if(alpha > max_alpha){
			max_alpha = alpha;
		}
		if(alpha < min_alpha){
			min_alpha = alpha;
		}
	}
	min_azimuth = 0.0f;
	max_azimuth = 360.0f;
	min_alpha = -25.0f;
	max_alpha = 5.0f;
	velo = true;
}

void rayTracer::rangeImagei3D(int WIDTH, int HEIGHT, cv::Mat &dstValue){
	cv::Mat dstImg(HEIGHT, WIDTH, CV_8UC1);
	bool interpolated = true;
	vector<std::pair<float,int>> distances;
	auto scloud = pVelocloud->points;

	for(int i=0; i<scloud.size(); i++){
		float px, py, pz;
		px = scloud[i].x;
		py = scloud[i].y;
		pz = scloud[i].z;

		float dst = sqrtf(pow(px-basePoint.x,2) + pow(py-basePoint.y,2) + pow(pz-basePoint.z,2));
		// Drop objects too far
		if(dst <= 35.0f)
		{
		std::pair<float,int> p;
		p.first = dst;
		p.second = i;
		distances.push_back(p);
		}
	}
	sort(distances.begin(), distances.end());
	float minD = distances[0].first;
	float maxD = distances[distances.size()-1].first;
	std::cout<<"Max distance: "<< maxD<<std::endl;
	std::cout<<"Min distance: "<< minD<<std::endl;

	// Sorted by distance
	scloudCpyV.clear();
	for (auto pair:distances){
		scloudCpyV.push_back(scloud[pair.second]);

	}

	int grid[WIDTH][HEIGHT]={0};
	float distgrid[WIDTH][HEIGHT]={0};

	// Horizontal resolution
	az_res = (max_azimuth - min_azimuth)/WIDTH;
	// Vertical resolution
	ang_res = (max_alpha - min_alpha)/HEIGHT;
	cout<<"Azimuth resolution: "<<az_res<<endl;
	cout<<"Angular resolution: "<<ang_res<<endl;

	cout<<scloudCpyV.size()<<endl;
	for(int i=0; i<scloudCpyV.size(); i++)
	{
		float px, py, pz;
		// Geometry
		px = scloudCpyV[i].x;
		py = scloudCpyV[i].y;
		pz = scloudCpyV[i].z;

		float dist = sqrtf( pow(px-basePoint.x,2) + pow(py-basePoint.y,2) + pow(pz-basePoint.z,2));
		float azimuth = calculateAz(px, py);
		float alpha = getAngle(scloudCpyV[i]);
		// Index into appropriate
		int index_az = ((azimuth-min_azimuth)/az_res+0.5f);
		int index_ang = ((alpha-min_alpha)/ang_res+0.5f);
		
		// Exception handling
		if(index_az == WIDTH)
			index_az = 0;
		if(index_ang == HEIGHT)
			index_ang = HEIGHT-1;
		if(index_az >= WIDTH || index_az < 0 || index_ang >= HEIGHT || index_ang < 0)
			continue;
		if(grid[index_az][index_ang] == 0){
			_points_min_dist[index_ang*WIDTH+index_az] = dist;
			grid[index_az][index_ang] +=1;
			distgrid[index_az][index_ang] = dist;
			_points[index_ang*WIDTH+index_az].push_back(&scloudCpyV[i]);
		}
	}
	
	for(int i = 0;i<HEIGHT;i++){
        for(int j = 0;j<WIDTH;j++){
			// Invalid depth values
			if(distgrid[j][HEIGHT - i -1] < 0.001f){
				distgrid[j][HEIGHT - i -1] = 100.0f;				
			}
			dstValue.at<float>(i,j) = distgrid[j][HEIGHT - i -1];
			if(dstValue.at<float>(i,j) < 0.001f)
				std::cout<<"Error";

		}
	}
	// interpolated = false;
	if(interpolated){
		interpolateDst(dstValue,WIDTH,HEIGHT);
	}

	// Only for visualization
	for(int i = 0;i<HEIGHT;i++){
        for(int j = 0;j<WIDTH;j++){
			if(dstValue.at<float>(i,j) > 90.0f || dstValue.at<float>(i,j) < 0.001f){
				dstImg.at<uchar>(i,j) = 0;
			}
			else{
            	dstImg.at<uchar>(i,j) = 255 - (uchar)(dstValue.at<float>(i,j)/MAXD*255);
			}
		}
	}
	cv::imshow("Velodyne Range Image", dstImg);
}
void rayTracer::rangeImageMLS(int WIDTH, int HEIGHT,cv::Mat &labelImg, cv::Mat &dstValue){
	vector<std::pair<float,int>> distances;
	auto scloud = pMlscloud->points;
	std::cout << "MLSTracing started..." << std::endl;

	for(int i=0; i<scloud.size(); i++){
		float px, py, pz;
		px = scloud[i].x;
		py = scloud[i].y;
		pz = scloud[i].z;
		float dst = sqrtf(pow(px-basePoint.x,2) + pow(py-basePoint.y,2) + pow(pz-basePoint.z,2));
		if(dst <= 35.0f)
		{
		std::pair<float,int> p;
		p.first = dst;
		p.second = i;
		distances.push_back(p);
		}
	}
	sort(distances.begin(), distances.end());
	float minD = distances[0].first;
	float maxD = distances[distances.size()-1].first;
	std::cout<<"Max distance: "<< maxD<<std::endl;
	std::cout<<"Min distance: "<< minD<<std::endl;

	// Sorted by distance

	scloudCpy.clear();
	for (auto pair:distances){
		scloudCpy.push_back(scloud[pair.second]);

	}
	int grid[WIDTH][HEIGHT]={0};
	int label[WIDTH][HEIGHT]={0};
	float distgrid[WIDTH][HEIGHT]={0};

	cout<<"Azimuth resolution: "<<az_res<<endl;
	cout<<"Angular resolution: "<<ang_res<<endl;

	cout<<scloudCpy.size()<<endl;
	for(int i=0; i<scloudCpy.size(); i++)
	{
		float px, py, pz;
		int pg;
		px = scloudCpy[i].x;
		py = scloudCpy[i].y;
		pz = scloudCpy[i].z;
		pg = scloudCpy[i].g;

		float dist = sqrtf( pow(px-basePoint.x,2) + pow(py-basePoint.y,2) + pow(pz-basePoint.z,2));
		float azimuth = calculateAz(px, py);
		float alpha = getAngle(scloudCpy[i]);

		int index_az = ((azimuth-min_azimuth)/az_res+0.5f);
		int index_ang = ((alpha-min_alpha)/ang_res+0.5f);
		if(index_az == WIDTH)
			index_az = 0;
		if(index_ang == HEIGHT)
			index_ang = HEIGHT-1;
		if(index_az >= WIDTH || index_az < 0 || index_ang >= HEIGHT || index_ang < 0)
			continue;
		if(grid[index_az][index_ang] == 0){
			grid[index_az][index_ang] +=1;
			distgrid[index_az][index_ang] = dist;
			// Vegetation
			if(pg == 108 || pg == 128){
				label[index_az][index_ang] = 1;
			}
		}
	}

	//  Handling invalid values
	for(int i = 0;i<HEIGHT;i++){
        for(int j = 0;j<WIDTH;j++){
			if(distgrid[j][HEIGHT - i -1] < 0.001f){
				distgrid[j][HEIGHT - i -1] = 100.0f;
			}
			dstValue.at<float>(i,j) = distgrid[j][HEIGHT - i -1];
			if(dstValue.at<float>(i,j) < 0.001f)
				std::cout<<"Error";
		}
	}
	

	// FOR VISUALIZATION
	// 0 - Hole
	// 1 - Vegetation
	// 2 - Not vegetation
	cv::Mat img(HEIGHT, WIDTH, CV_8UC1);
	cv::Mat imgVeg(HEIGHT, WIDTH, CV_8UC1);
	for(int i = 0;i<HEIGHT;i++){
        for(int j = 0;j<WIDTH;j++){
			if(dstValue.at<float>(i,j) > 90.0f){
				img.at<uchar>(i,j) = 0;
			}
			else{
            	img.at<uchar>(i,j) = 255 - (uchar)(dstValue.at<float>(i,j)/MAXD*256);
			}
			labelImg.at<uchar>(i,j) = 255;
			imgVeg.at<uchar>(i,j) = 0;
			if(label[j][HEIGHT - i - 1] == 1){
				labelImg.at<uchar>(i,j) = 0;
				imgVeg.at<uchar>(i,j) = 100;
			}
		}
	}
	cv::imshow("MLS Range Image", img);
	cv::imshow("MLS Vegetation Map", imgVeg);
	cv::waitKey(0);

}
void rayTracer::interpolateDst(cv::Mat &dstValue,int WIDTH, int HEIGHT){
	int N = 1;
	for(int i = N;i<HEIGHT-N;i++){
        for(int j = N;j<WIDTH-N;j++){
			float distanceValue = dstValue.at<float>(i,j);
			if(distanceValue > 90.0f || distanceValue < 0.1f){
				vector<float> neighbour_dist;
				int cnt = 0;
				float min_v = 500.0f;
				float max_v = 0.0f;
				for (int k = -1;k<2;k++){
					for (int kj = -1;kj<2;kj++){
						float val = dstValue.at<float>(i+k,j+kj);
						if (val < 90.0f && val > 0.001f){
							cnt++;
							neighbour_dist.push_back(val);
							if (val > max_v)
								max_v = val;
							if(val < min_v)
								min_v = val;
						}
					}
				}
				if(cnt >= 6 && (max_v - min_v) < 2.0f){
					float average = std::accumulate( neighbour_dist.begin(), neighbour_dist.end(), 0.0)/neighbour_dist.size();
					dstValue.at<float>(i,j) = average;
				}
			}
		}
	}
}


#ifndef SCENEINTERPRETER_H
#define SCENEINTERPRETER_H

#include "DataModel.h"

#include <queue>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>


struct cell {
    pcl::PointCloud<pcl::PointXYZRGBNormal*> points;
    std::vector<int> index;
    
    std::vector<TData*> data;
    pcl::PointXYZ min, max;
    int id = -1;
    int cluster = -1;
    int pos = -1;
    double localGroundHeight = 0;
    int maxHistSegment = -1;
    int maxSegmentIdx = -1;
    bool visited = false;
    bool vis = false;
    
    int act;
    int posx, posz;

    std::vector<pcl::PointXYZRGB> colors;
};

//////////////////////////////////////////////////////

struct MyObject {
    pcl::PointCloud<pcl::PointXYZRGBNormal*> _points;
    std::vector<int> index;
    int id = -1;
    pcl::PointXYZ min, max;
    size_t cloudSize = 0;
    size_t actualCloudSize = 0;
    size_t cells = 0;
    int cluster = -1;
    double maxHeight;
    double distance = 0;
    bool matched = false;
    Eigen::Vector3f normal;
    
    std::vector<pcl::PointXYZ> bb;//bounding box
	float score; //Point Pillar confidence
	int objclass;	// label: 0 car, 1 bike, 2 pedestrian


    void XY(float& x, float& y) {
        x = (min.x + max.x) * 0.5f;
        y = (min.y + max.y) * 0.5f;
    }

    float Z() { return (max.z - min.z); }
    float X() { return (max.x - min.x); } 
    float Y() { return (max.y - min.y); }

    float V() { return (Z() * X() * Y()); } 
    
    int size() { return _points.points.size();} 
     
};

//////////////////////////////////////////////////////

class SceneIP {


protected:
    pcl::PointXYZRGBNormal min, max;

    float wallSize = 2.1f;
    float wallHeightDifference = 2.1f;
    float groundSize = 0.25f;
    float voxelSize = 0.2f;
    float sensorHeight = 1.95;

    size_t voxnumx, voxnumy, voxnumz;
    int globalClass;

    virtual void _objectDetection() = 0;
    virtual bool _buildScene() = 0;

public:
    SceneIP();
    virtual ~SceneIP();

    void setModel(boost::shared_ptr<DataModel>);    
    void setVoxelSize(float);
    void setWallSize(float);
    void setWallHeightDifference(float );   
    void setGroundSize(float);
    void setSensorHeight(float);

    float getSensorHeight() { return sensorHeight; }

    virtual bool buildScene() = 0;
    virtual void segmentScene() = 0;
    virtual void objectDetection() = 0;

    virtual void reset() = 0;

    std::vector<MyObject> objects;
    std::vector<MyObject> objects2; 
    boost::shared_ptr<DataModel> model;
};

//////////////////////////////////////////////////////

class SceneIP2D : public SceneIP {
  

    void _objectDetection();
    bool _buildScene();
    bool inline addPoint(pcl::PointXYZRGBNormal &_point, size_t idx);      

public:
    SceneIP2D();
    virtual ~SceneIP2D();

    std::vector<std::vector<cell>> scene;
    std::vector<pcl::PointXY> objectXY1;    

    bool buildScene();
    void segmentScene();
    void objectDetection();
    void detectGround();

    bool is_segmented = false;

    void colorSegment();
    void colorDetect();

    void reset();


};

bool SceneIP2D::addPoint (pcl::PointXYZRGBNormal &_point, size_t idx) {
    
    float currx, curry;
    //std::cout<<model->min.x<<std::endl;
    // let's calculate which grid cell this point goes in
    
    currx = _point.x - model->min.x;
    curry = _point.y - model->min.y;
    
    int posx = floor(currx / voxelSize);
    int posy = floor(curry / voxelSize);
    
    if(posx > -1 && posx < voxnumx && posy > -1 && posy < voxnumy) {    
        pcl::PointXYZRGBNormal* _ptr = &_point;
        scene[posx][posy].points.points.push_back(_ptr);
        scene[posx][posy].index.push_back(idx);        
    
        if (scene[posx][posy].min.x > _point.x)
            scene[posx][posy].min.x = _point.x;
        if (scene[posx][posy].max.x < _point.x)
            scene[posx][posy].max.x = _point.x;
        if (scene[posx][posy].min.y > _point.y) 
            scene[posx][posy].min.y = _point.y;
        if (scene[posx][posy].max.y < _point.y)
            scene[posx][posy].max.y = _point.y;
        if (scene[posx][posy].min.z > _point.z)
            scene[posx][posy].min.z = _point.z;
        if (scene[posx][posy].max.z < _point.z)
            scene[posx][posy].max.z = _point.z;
    }
    return true;
}


#endif
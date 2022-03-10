#ifndef DATAMODEL_H
#define DATAMODEL_H

#include <vector>
#include <boost/smart_ptr/make_shared.hpp>
#include <pcl/common/common.h>
#include <unordered_map>
#include <future>
#include <chrono>

struct TData {
	int cl = 0;
	float intensity;
	bool state;
	int object_id;
	int class_id;
	float azimuth;
	float distance;
	bool loaded = false;
	int track;
};
enum PointClasses {
	GROUND = 1,
	SPARSE = 2,
	WALL = 3,
	OBJECT = 4,
	INTENSITY_MARK = 5,
	VEGETATION = 6,
};

class AbstractDataModel {

public:
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> spCloud;
	boost::shared_ptr<std::vector<TData>> spData;

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> spCloud2;
	boost::shared_ptr<std::vector<TData>> spData2;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> spCloudMLS;
	pcl::PointXYZRGBNormal min, max;
	pcl::PointXYZRGBNormal min2, max2;
	pcl::PointXYZRGBNormal orig_min, orig_max;	

	virtual bool readData(std::string, int) = 0;
};

class DataModel :public AbstractDataModel {

public:
	DataModel();
	virtual ~DataModel();

	std::unordered_map<std::string, std::pair<float, float>> gpsMap;
	void loadGPSTOIMG(std::string path);
	

	//////////////////////////////////////////////////////////////////////////
	/// Load pointcloud data

	bool readPcd(std::string, int);
	bool readLas(std::string, int);
	virtual bool readData(std::string, int);

	void log(std::string _in);
	bool IsMessy() { bool m = modelMessy; modelMessy = false; return m; }
	void setModelMessyTrue();

	bool spCloudIsLoaded = false;
	bool spCloud2IsLoaded = false;
	int erasePointNum = 500;

	//////////////////////////////////////////////////////////////////////////
	/// Export functions

	void exportToPCD(bool);

	//////////////////////////////////////////////////////////////////////////

private:
	std::string extension;
	std::string file;
	bool modelMessy;
};

#endif

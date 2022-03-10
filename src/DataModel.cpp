#include "DataModel.h"
#include <vector>
#include <boost/smart_ptr/make_shared.hpp>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

////////////////////// DataModel //////////////////////

DataModel::DataModel() : modelMessy(false) {
	spCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>(pcl::PointCloud<pcl::PointXYZRGBNormal>());
	spData = boost::make_shared<std::vector<TData>>(std::vector<TData>());

	spCloud2 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>(pcl::PointCloud<pcl::PointXYZRGBNormal>());
	spData2 = boost::make_shared<std::vector<TData>>(std::vector<TData>());

	spCloudMLS = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>(pcl::PointCloud<pcl::PointXYZRGBNormal>());
	loadGPSTOIMG("/home/balazs/AIPro-Localizer/gps.eov");
}

DataModel::~DataModel() {
	spCloud = nullptr;
	spCloud2 = nullptr;
	spData = nullptr;
	spData2 = nullptr;
}

//************************************
// Author:	  Balazs Nagy
// Method:    readPcd
// FullName:  DataModel::readPcd
// Access:    public
// Returns:   bool
// Qualifier:
// Parameter: std::string _input
// Parameter: int chooser
// Mark:	  Depending on chooser state it reads point cloud into spCloud or spCloud2 from .pcd file format.
//************************************
bool DataModel::readPcd(std::string _input, int chooser) {
	pcl::PCDReader reader;

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> tmpCloud;
	boost::shared_ptr<std::vector<TData>> tmpData;
	pcl::PointXYZRGBNormal* _min;
	pcl::PointXYZRGBNormal* _max;

	TData _t;

	if (!chooser) {
		spCloud->clear(); spCloud->resize(0);
		tmpCloud = spCloud;
		tmpData = spData;
		_min = &min;
		_max = &max;
		spCloudIsLoaded = true;
	}
	else {
		spCloud2->clear(); spCloud2->resize(0);
		tmpCloud = spCloud2;
		tmpData = spData2;
		_min = &min2;
		_max = &max2;
		spCloud2IsLoaded = true;
	}

	reader.read(_input, *tmpCloud);

	tmpData->resize(tmpCloud->points.size());

	if (tmpCloud->points.size()) {
		// increasing order
		std::sort(tmpCloud->points.begin(), tmpCloud->points.end(), [](pcl::PointXYZRGBNormal e1, pcl::PointXYZRGBNormal e2){ return e1.z < e2.z; });
		tmpCloud->points.erase(tmpCloud->points.begin(), tmpCloud->points.begin() + (tmpCloud->points.size()*0.05));

		pcl::getMinMax3D(*tmpCloud, *_min, *_max);

		for (auto i = 0; i < tmpData->size(); ++i) {
			_t.cl = 0;
			_t.state = 1;
			_t.intensity = 0;
			tmpData->at(i) = _t;
		}

		pcl::getMinMax3D(*tmpCloud, *_min, *_max);

		min.x = _min->x;
		min.y = _min->y;
		min.z = _min->z;

		max.x = _max->x;
		max.y = _max->y;
		max.z = _max->z;

		tmpCloud = nullptr;
		tmpData = nullptr;

		return true;
	}
	tmpCloud = nullptr;
	tmpData = nullptr;

	return false;
}


void DataModel::setModelMessyTrue(){
	modelMessy = true;
}

bool DataModel::readData(std::string _file, int chooser) {
	extension = _file.substr(_file.find_last_of("."));

	if (extension == ".pcd") {
		bool b = readPcd(_file, chooser);
		file = _file;
		modelMessy = true;
		return b;
	}
	else
		return false;
}

void DataModel::log(std::string _in) {
	std::ofstream o("output\\datamodel_Log.txt", std::ios::app);
	o << _in << "\n";
	o.close();
}

//////////////////////////////////////////////////////////////////////////
/// Export functions

//************************************
// Method:    exportToPCD
// FullName:  DataModel::exportToPCD
// Access:    public
// Returns:   void
// Qualifier:
// Parameter: bool binary
//************************************
void DataModel::exportToPCD(bool binary = true) {
	pcl::PCDWriter writer;
	spCloud->width = spCloud->points.size();
	spCloud->height = 1;
	writer.write(file + "a.pcd", *spCloud, true);
	std::cout << "Exporting PCD file to : " << file + ".pcd" << std::endl;
}

void DataModel::loadGPSTOIMG(std::string path) {

	std::ifstream ifs(path);

	std::vector<std::string> tokens;
	std::string line = "";

	double w, h;
	std::string tick;

	double w1 = 0.0, h1 = 0.0;

	int num = 0;
	while(std::getline(ifs, line)) {
		tokens.clear();
		boost::algorithm::split(tokens, line, boost::algorithm::is_any_of(","));

		w1 = 0.0, h1 = 0.0;

		if(tokens.size() == 3 && tokens[0] != "" && tokens[1] != "" && tokens[2] != "") {

			tick = tokens[0];
			std::istringstream(tokens[1]) >> w;
			std::istringstream(tokens[2]) >> h;




			std::pair<float, float> f(w, h);
			std::pair<std::string, std::pair<float, float>> ff(tick, f);

			gpsMap.insert(ff);
		}
	}
}
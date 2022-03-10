#include "sceneInterpreter.h"

SceneIP::SceneIP() {
    model = boost::make_shared<DataModel>(DataModel());
}

SceneIP::~SceneIP() {
    model = nullptr;
}

void SceneIP::setModel(boost::shared_ptr<DataModel> _model) {
    model = _model;
} 

void SceneIP::setVoxelSize(float s) {
    voxelSize = s;
}

void SceneIP::setWallSize(float s) {
    wallSize = s;
}

void SceneIP::setWallHeightDifference(float s) {
    wallHeightDifference = s;
}

void SceneIP::setGroundSize(float s) {
    groundSize = s;
}

void SceneIP::setSensorHeight(float s) {
    sensorHeight = s;
}

///////////// SIP2D ///////////////////////////////////

SceneIP2D::SceneIP2D() {
    voxelSize = 0.2f;
}

SceneIP2D::~SceneIP2D() {
    model = nullptr;    
}

void SceneIP2D::_objectDetection() {
    globalClass = -1;
	std::queue<cell*> floodfillQueue;

    for (int i = 0; i < voxnumx; ++i) {
        for (int a = 0; a < voxnumy; ++a) {
            //if(scene[i][a].max.z < ( (-1*sensorHeight) + 1.1 )) continue;
			
			if (scene[i][a].points.size() > 0 && !scene[i][a].visited /*&& scene[i][a].cluster != WALL*/ && scene[i][a].cluster != GROUND && scene[i][a].cluster != SPARSE) { 
                floodfillQueue.push(&scene[i][a]);

                scene[i][a].id = ++globalClass;
                scene[i][a].visited = true;
                
                MyObject obj;
                obj.cloudSize += scene[i][a].points.size();
                obj.id = globalClass;
                obj.min.x = obj.min.y = obj.min.z = INT_MAX;
                obj.max.x = obj.max.y = obj.max.z = INT_MIN;
                ++obj.cells;
                
                objects.push_back(obj);
				
				while (!floodfillQueue.empty()) {
				    for (int j = -1; j < 2; ++j) {
					    for (int k = -1; k < 2; ++k) {
						    int posX = floodfillQueue.front()->posx;
                            int posZ = floodfillQueue.front()->posz;
                            int posXJ = posX+j;
                            int posZK = posZ+k;

						    if ((posX + j) > -1 && (posX + j) < voxnumx && (posZ + k) > -1 && (posZ + k) < voxnumy && /*scene[posXJ][posZK].cluster != WALL &&*/ scene[posXJ][posZK].cluster != GROUND && scene[posXJ][posZK].cluster != SPARSE) {
                               // if(scene[posXJ][posZK].max.z < ( (-1*sensorHeight) + 1.1 )) continue;
							
							    if (scene[posXJ][posZK].points.size() > 0 && !scene[posXJ][posZK].visited ) { 									
                                    if (j == -1 || j == 1 || k == -1 || k == 1) {
                                        floodfillQueue.push(&scene[posXJ][posZK]);
                                        scene[posXJ][posZK].visited = true; 
                                        scene[posXJ][posZK].id = globalClass; 
                                        objects[globalClass].cloudSize += scene[posXJ][posZK].points.size();

                                        if (scene[posXJ][posZK].min.x < objects[globalClass].min.x)
                                            objects[globalClass].min.x = scene[posXJ][posZK].min.x;
                                        if (scene[posXJ][posZK].max.x > objects[globalClass].max.x)
                                            objects[globalClass].max.x = scene[posXJ][posZK].max.x;
                                        if (scene[posXJ][posZK].min.y < objects[globalClass].min.y)
                                            objects[globalClass].min.y = scene[posXJ][posZK].min.y;
                                        if (scene[posXJ][posZK].max.y > objects[globalClass].max.y)
                                            objects[globalClass].max.y = scene[posXJ][posZK].max.y;
                                        if (scene[posXJ][posZK].min.z < objects[globalClass].min.z)
                                            objects[globalClass].min.z = scene[posXJ][posZK].min.z;
                                        if (scene[posXJ][posZK].max.z > objects[globalClass].max.z)
                                            objects[globalClass].max.z = scene[posXJ][posZK].max.z;
                                        
                                        ++objects[globalClass].cells;
                                    }
								}
							}
						}
					}
					floodfillQueue.pop();
				}
			}
		}
    }

    for (int i = 0; i < voxnumx; ++i) {
        for (int a = 0; a < voxnumy; ++a) {
            if (scene[i][a].points.size() > 0) {
                scene[i][a].visited = false;

                if (scene[i][a].id > -1 && scene[i][a].id < objects.size()) {
                    std::copy(scene[i][a].points.begin(), scene[i][a].points.end(), std::back_inserter(objects[scene[i][a].id]._points.points));
                    std::copy(scene[i][a].index.begin(), scene[i][a].index.end(), std::back_inserter(objects[scene[i][a].id].index));
                }
            }
        }
    }
}

bool SceneIP2D::_buildScene() {
    scene.clear();
    model->min.x = -35.0;
    model->min.y = -35.0;
    model->min.z = -10.0;
    model->max.x = 35.0;
    model->max.y = 35.0;
    model->max.z = 10.0;
    float _rangeX = model->max.x - model->min.x;
    float _rangeY = model->max.y - model->min.y;
    voxnumx = ceil(float(_rangeX / voxelSize));
    voxnumy = ceil(float(_rangeY / voxelSize));
    
    scene.reserve(voxnumx);
    cell tmpcell;
    tmpcell.min.y = tmpcell.min.z = tmpcell.min.x = INT_MAX;
    tmpcell.max.y = tmpcell.max.z = tmpcell.max.x = INT_MIN;
    tmpcell.act = -1;
    for (size_t i = 0; i <= voxnumx; ++i) {
        std::vector<cell> tmp;
        tmp.reserve(voxnumy);
        for (size_t j = 0; j < voxnumy; ++j) {
            tmpcell.posx = i; 
            tmpcell.posz = j;
            tmp.push_back(tmpcell);
        }
        scene.push_back(tmp);
    }
    for (int i = 0; i < voxnumx; ++i) {
        for (int j = 0; j < voxnumy; ++j) {          
            scene[i][j].points.points.clear();
            scene[i][j].points.resize(0);
            scene[i][j].index.clear();
            

            scene[i][j].max.x = scene[i][j].max.y = scene[i][j].max.z = -35;
            scene[i][j].min.x = scene[i][j].min.y = scene[i][j].min.z = 35;

            scene[i][j].id = -1;
            scene[i][j].cluster = -1;
            scene[i][j].pos = -1;
            scene[i][j].localGroundHeight = 0;
            scene[i][j].maxHistSegment = -1;
            scene[i][j].maxSegmentIdx = -1;
            scene[i][j].visited = false;
            scene[i][j].vis = false;
            scene[i][j].act = -1;  
        }
    }

    for (size_t i = 0; i < model->spCloud->points.size(); i++) {
       addPoint(model->spCloud->points[i], i);
    }
    return true;
}

bool SceneIP2D::buildScene() {
    if (_buildScene()) {
        detectGround();
        return true;
    }
    return false;
}

void SceneIP2D::segmentScene() { 
    for (int i = 0; i < voxnumx; ++i) {
        for (int j = 0; j < voxnumy; ++j) {          
            if(scene[i][j].points.points.size() > 0) {
                float distance =  sqrtf(scene[i][j].points[0]->x * scene[i][j].points[0]->x + scene[i][j].points[0]->y * scene[i][j].points[0]->y);

                    if ( (scene[i][j].max.z - scene[i][j].min.z ) < groundSize && scene[i][j].max.z < scene[i][j].localGroundHeight + 0.25 /*+distance/15.0*/)
                        scene[i][j].cluster = GROUND;
                    else if( (scene[i][j].max.z - scene[i][j].min.z ) > wallHeightDifference || scene[i][j].max.z > (-1.0*sensorHeight + wallSize) )
                        scene[i][j].cluster = WALL;
                    else
                        scene[i][j].cluster = OBJECT;

            }
            else {
                scene[i][j].cluster = SPARSE; 
            }
           
        }
    }

    for (int i = 0; i < voxnumx; ++i) {
        for (int j = 0; j < voxnumy; ++j) {
            if(scene[i][j].localGroundHeight > 0)
            if(scene[i][j].cluster == OBJECT) {
                int c = 0;
                for (int k = -4; k < 5; ++k) {
                    for (int l = -4; l < 5; ++l) {
                        if( (i + k) > -1 && (i + k) < voxnumx && (j + l) > -1 && (j + l) < voxnumy ) {
                            if(scene[i+k][j+l].cluster == WALL)
                                ++c;
                        }
                    }
                }
                if(c > 0 && (scene[i][j].max.z - scene[i][j].min.z) > 1.4 )
                    scene[i][j].cluster = WALL;  
            }
        }
    }
}

void SceneIP2D::objectDetection() {
    if(!is_segmented)
        segmentScene();
    objects.clear();
    objects.resize(0);
    _objectDetection();

}

void SceneIP2D::colorSegment() {
    for (int i = 0; i < voxnumx; ++i) {
        for (int j = 0; j < voxnumy; ++j) {
          	if (scene[i][j].cluster == GROUND) {
    		    for (auto p : scene[i][j].points.points) {
    			    p->r = 255;
    			    p->g = 255;
    			    p->b = 255;
    		    }
    	    }/*
            else if(scene[i][j].cluster == WALL) {
                for (auto p : scene[i][j].points.points) {
    			    p->r = 255.0;
    			    p->g = 0.0;
    			    p->b = 0.0;
    		    }
            }
            else if(scene[i][j].cluster == OBJECT) {
                for (auto p : scene[i][j].points.points) {
    			    p->r = 0.0;
    			    p->g = 255.0;
    			    p->b = 0.0;
    		    }
            }
            else {
                for (auto p : scene[i][j].points.points) {
    			    p->r = 0.0;
    			    p->g = 0.0;
    			    p->b = 0.0; 
    		    }
            }*/
        }
    }
    model->setModelMessyTrue();    
}

void SceneIP2D::colorDetect() {
    for (size_t i = 0; i < objects2.size(); ++i) {
        int r = rand() % 255;
        int g = rand() % 255;
        int b = rand() % 255;

        if(objects2.at(i)._points.size() > 0) {
            for (size_t j = 0; j < objects2.at(i)._points.size(); ++j) {
                objects2.at(i)._points.at(j)->r = r;
                objects2.at(i)._points.at(j)->g = g;
                objects2.at(i)._points.at(j)->b = b;
            }
        }
    }
    model->setModelMessyTrue();
}

void SceneIP2D::detectGround() {
    float currx, currz;
    int posx, posz;

    if(model->orig_min.x < 0 || model->orig_min.y < 0) {
        currx = 0.0 - model->orig_min.x;
        currz = 0.0 - model->orig_min.y;
    }
    else {
        currx = 0.0 + model->orig_min.x;
        currz = 0.0 + model->orig_min.y;
    }
    
    posx = floor(currx / voxelSize);
    posz = floor(currz / voxelSize);
    
    std::queue<cell*> floodfillQueue;
    floodfillQueue.push(&scene[posx][posz]);
    scene[posx][posz].vis = true;
    scene[posx][posz].localGroundHeight = (-1*model->orig_min.z)-sensorHeight;
    int prevNonNullCellx = posx;
    int prevNonNullCelly = posz;
    
    while (!floodfillQueue.empty()) {
        double height = 0;
        for (int j = -1; j < 2; ++j) {
            for (int k = -1; k < 2; ++k) {
  
                int posX = floodfillQueue.front()->posx;
                int posZ = floodfillQueue.front()->posz;
                int posXJ = posX+j;
                int posZK = posZ+k;
  
                if (posXJ < voxnumx && posXJ > -1 && posZK < voxnumy && posZK > -1 && posX < voxnumx && posX > -1 && posZ < voxnumy && posZ > -1) {

                    if (scene[posX][posZ].points.size() > 0) {
                        prevNonNullCellx = posX;
                        prevNonNullCelly = posZ;
                    }
                    
                    if (scene[posXJ][posZK].points.size() > 0 && !std::isnan(scene[posXJ][posZK].min.z)) {
                        height = fabs(scene[posXJ][posZK].min.z - scene[posX][posZ].localGroundHeight);
                        if (height < 0.2) {
                            scene[posXJ][posZK].localGroundHeight = scene[posXJ][posZK].min.z;
                        }
                        else {
                            scene[posXJ][posZK].localGroundHeight = scene[posX][posZ].localGroundHeight;
                        }
                    }
                    else {
                        scene[posXJ][posZK].localGroundHeight = scene[prevNonNullCellx][prevNonNullCelly].localGroundHeight;
                    }
        
                    if (!scene[posXJ][posZK].vis && (j == -1 || j == 1 || k == -1 || k == 1)) {
                        floodfillQueue.push(&scene[posXJ][posZK]);
                        scene[posXJ][posZK].vis = true;
                    }
                }
            }
        }
  
        floodfillQueue.pop();
    }
}

void SceneIP2D::reset() {
    scene.clear();
    objects.clear();
    objects.resize(0);
}
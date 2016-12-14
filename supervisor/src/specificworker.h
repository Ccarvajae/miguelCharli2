/*
 *    Copyright (C) 2016 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	
	void newAprilTag(const tagsList &tags);

	void waitM();
	void searchM();  
	void irAMarca(int marca);
	
	
public slots:
	void compute(); 	

private:
   struct Tag{
    mutable QMutex m;
    InnerModel *inner;
    int id;
    QVec poseAnt = QVec::zeros(3);
    QVec pose = QVec::zeros(3);
	
    QVec getPose(){
      QMutexLocker lm(&m);
      return pose;
    }
	
    void init(InnerModel *innermodel){
      inner = innermodel;
    }
	
    void copy(float x, float z, int id_){
      QMutexLocker lm(&m);
      qDebug()<<"COORDENADAS RESPECTO A LA CAMARA: X: "<<x<<"    Z: "<<z;
      pose = inner->transform("world", QVec::vec3(x,0,z),"rgbd");	//base
      qDebug()<<"COORDENADAS RESPECTO A LA SIMULACION: X: "<< pose.x()<<"   Z: "<<pose.z();	
      id = id_;
    }
	
    int getID(){
      QMutexLocker lm(&m);
      return id;
    }
	
    bool changed(){
      QMutexLocker lm(&m);      
      float d = (pose - poseAnt).norm2();
      poseAnt = pose;
      return d > 100;
    }
  };
  
  
  Tag tag;    
  
  enum class State {WAIT,SEARCH, MARK0, MARK1, MARK2, MARK3};
  State state = State::SEARCH;
  InnerModel* innermodel;
  int contTags = 0;
  
  
  
};

#endif


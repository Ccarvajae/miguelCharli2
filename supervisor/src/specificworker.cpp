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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{	
    innermodel = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
    timer.start(Period);	
    tag.init(innermodel);
    return true;
}

void SpecificWorker::compute(){

    //gotopoint_proxy->go("", 1000, 2000, 0);
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState ( bState );
    innermodel->updateTransformValues ("base",bState.x, 0,bState.z, 0,bState.alpha, 0); 
  
    switch(state){
      case State::WAIT:
	cout<<"WAIT"<<endl;
	waitM();
      break;
      case State::SEARCH:
	cout<<"SEARCH"<<endl;
	searchM();
      break;
      case State::MARK0:
	cout<<"MARK0"<<endl;
	irAMarca(0);
      break;
      case State::MARK1:
	cout<<"MARK1"<<endl;
	irAMarca(1);
      break;
      case State::MARK2:
	cout<<"MARK2"<<endl;
	irAMarca(2);
      break;
      case State::MARK3:
	cout<<"MARK3"<<endl;
	irAMarca(3);
    break;  
  }	    
}

void SpecificWorker::newAprilTag(const tagsList &tags)
{
  int i;
  
  cout<<"VEO       "<<tags.size()<<"      SEÑALES"<<endl<<endl;
  
  for(i = 0; i < tags.size(); i++){
    cout<<"Identidad:"<<tags[i].id<<endl;
    cout<<"TX:"<<tags[i].tx<<endl;
    cout<<"TY:"<<tags[i].ty<<endl;
    cout<<"TZ:"<<tags[i].tz<<endl;
    cout<<"RX:"<<tags[i].rx<<endl;
    cout<<"RY:"<<tags[i].ry<<endl;
    cout<<"RZ:"<<tags[i].rz<<endl;
    
    tag.copy(tags[0].tx, tags[0].tz, tags[0].id);
  }
}

void SpecificWorker::waitM(){
  
  if( gotopoint_proxy->atTarget() == true){
    differentialrobot_proxy->stopBase();
    state = State::SEARCH;
  }
  else if(tag.changed()){
    qDebug()<<tag.getPose().size();
    qDebug()<<tag.getPose().x();
    qDebug()<<tag.getPose().z();
    gotopoint_proxy->go("",tag.getPose().x(),tag.getPose().z(), 0);
  }
  qDebug()<<"ADIOS ADIOS";
}

void SpecificWorker::searchM(){

  qDebug()<<contTags<< " NUMERO DE TAG:  "<<tag.getID();

  if(tag.getID()==contTags){
     differentialrobot_proxy->stopBase();
     gotopoint_proxy->go("",tag.getPose().x(),tag.getPose().z(), 0);
     
     switch(contTags){
      case 0:
	state = State::MARK0;
	break;
      case 1: 
	state = State::MARK1;
	break;
      case 2: 
	state = State::MARK2;
	break;
      case 3: 
	state = State::MARK3;
	break;
     }     
  }
  
  differentialrobot_proxy->setSpeedBase(0,0.3);
}

void SpecificWorker::irAMarca(int marca){

 switch(marca){
    case 0: 
    case 1:
    case 2:
	contTags++;
	state = State::WAIT;     
	qDebug()<<"LLEGADA A MARCA, SE BUSCA LA SIGUIENTE MARCA CON EL ID :"<<contTags;
    break;  
    case 3:
      contTags = 0;
      state = State::WAIT;     
      qDebug()<<"FINALIZADA LA TAREA, SE REINICIA EL SISTEMA Y VUELVE A EMPEZAR";
    break;
 }
}
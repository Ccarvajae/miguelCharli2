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
    return true;
}

void SpecificWorker::compute(){

    //gotopoint_proxy->go("", 1000, 2000, 0);
  
  switch(state){
    case State::INIT:
	cout<<"INIT";
	initM();
    break;
    case State::SEARCH:
	cout<<"SEARCH";
	searchM();
    break;
    case State::MARK0:
	cout<<"MARK0";
	irAMarca(0);
    break;
    case State::MARK1:
	cout<<"MARK1";
	irAMarca(1);
    break;
    case State::MARK2:
	cout<<"MARK2";
	irAMarca(2);
    break;
    case State::MARK3:
	cout<<"MARK3";
	irAMarca(3);
    break;  
  }
}

void SpecificWorker::newAprilTag(const tagsList &tags)
{
  qDebug<<tags[0];
  
}

void SpecificWorker::initM(){

  
}

void SpecificWorker::searchM(){

  
}

void SpecificWorker::irAMarca(int marca){

  
}
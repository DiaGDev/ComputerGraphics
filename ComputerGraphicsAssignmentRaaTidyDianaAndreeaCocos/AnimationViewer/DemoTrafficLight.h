#pragma once

#include <TrafficSystem/TrafficLightFacade.h>

class DemoTrafficLight: public TrafficSystem::TrafficLightFacade
{
public:
	DemoTrafficLight(std::string sName, osg::Node* pAsset, osg::Matrixf m, bool bVisible);
	virtual ~DemoTrafficLight();

	virtual osg::Vec3f getFacadeCollisionPoint() override;

};


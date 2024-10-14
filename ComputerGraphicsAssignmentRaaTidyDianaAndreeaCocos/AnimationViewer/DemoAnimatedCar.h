#pragma once

#include <windows.h>
#include <osg/AnimationPath>
#include <TrafficSystem/CarFacade.h>

class DemoAnimatedCar : public TrafficSystem::CarFacade
{
public:
	DemoAnimatedCar(std::string sName, osg::Node* pAsset, osg::Matrix m, bool bVisible);
	virtual ~DemoAnimatedCar();

	void setAnimationPath(osg::AnimationPath* pPath);

	virtual bool run(osg::Object* object, osg::Object* data); // inherted from the osg::callback (in TrafficSystem::CarFacade

	virtual osg::Vec3f getFacadeCollisionPoint() override; // inhereted from collision target (cars can have collision targets too


protected:
	osg::MatrixTransform* m_pAnimationTransform; // needed for the animation path. Needs to be inserted between the root and transform nodes
	osg::MatrixTransform* m_pCollisionTarget; // defines the space transformation for the collision volume
};

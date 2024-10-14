#include <windows.h>
#include <iostream>
#include <osg/Geode>

#include <TrafficSystem/TrafficLightFacade.h>
#include <Assignment/ControllableTrafficLightFacade.h>

#include "SelectionHandler.h"
#include "DemoAnimatedCar.h"
#include "DemoTrafficLight.h"


DemoAnimatedCar::DemoAnimatedCar(std::string sName, osg::Node* pAsset, osg::Matrix m, bool bVisible): TrafficSystem::CarFacade(sName, pAsset, m, bVisible), m_pAnimationTransform(new osg::MatrixTransform()), m_pCollisionTarget(new osg::MatrixTransform()), m_pCamera(0)
{
	// insert the animation transform inot the facade sub tree to allow it to control an empty space transform
	// Take note of the order of these operations. This avoids the existing objects being unref() and deleted
	m_pAnimationTransform->addChild(m_pTransformation);
	m_pRoot->removeChild(m_pTransformation);
	m_pRoot->addChild(m_pAnimationTransform);

	// set up the position and size of the detector box
	// remeber this is in the coordinate frame of the car

	toggleVisible(); // uncomment to hide the detector box
	setTransform(osg::Matrixf::translate(1.0f, -2.5f, -0.7f));
	setBound(osg::Vec3f(4.0f, 2.0f, 0.8f));


	SelectionHandler* pSH = new SelectionHandler(this);
	m_pRoot->setUserData(pSH);

	// add a collision detector to the back of the car - we will not be using this, but can be used to stop cars running into each other
	m_pCollisionTarget->addChild(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3f(0.0f, 0.0f, 0.0f), 20.0f)));
	m_pCollisionTarget->setMatrix(osg::Matrix::translate(-110.0f, 0.0f, 0.0f));
	m_pAnimationTransform->addChild(m_pCollisionTarget);
}

DemoAnimatedCar::~DemoAnimatedCar()
{
}

void DemoAnimatedCar::setAnimationPath(osg::AnimationPath* pPath)
{
	// add the animation path to the facade - connect it as a callback to the new animation transform
	osg::AnimationPathCallback* pAPC = new osg::AnimationPathCallback(pPath);
	m_pAnimationTransform->setUpdateCallback(pAPC);
}

//this is the callback fundtion - called on each update traversal
bool DemoAnimatedCar::run(osg::Object* object, osg::Object* data)
{
	bool bStop = false;
	
	// used to detect the world space position onn the collision taregts and text for inclusion in the detector volume

	// get the world to local matrix for the collide volume
	// remeber this is an animated object so this will change for every frame
	// and, therefore, need recalculating each time we use it
	osg::Matrix mW2L = osg::computeWorldToLocal(m_pPosition->getParentalNodePaths(0)[0]);

	// loop through all the facades to find those that implment the collision target interface, remember this is a map 
	for(Common::FacadeMap::iterator it=facades().begin();it!=facades().end();it++)
	{
		// use dynamic cast to determine whether this object implements the collision target interface
		if(TrafficSystem::CollisionTarget* pCT=dynamic_cast<TrafficSystem::CollisionTarget*>(it->second))
		{
			if (DemoAnimatedCar* pDemoAnimatedCar = dynamic_cast<DemoAnimatedCar*>(pCT))
			{
				osg::Vec3f vTargetPosition = pCT->getFacadeCollisionPoint();

				vTargetPosition = vTargetPosition * mW2L;
				if (m_pGeode->getBoundingBox().contains(vTargetPosition))
				{
					//std::cout << "Ouch, I have hit a traffic light: " << pTrafficLightFacade->root()->getName() << std::endl;

					if (DemoAnimatedCar* pDAC = dynamic_cast<DemoAnimatedCar*>(pDemoAnimatedCar))
					{
						if (osg::AnimationPathCallback* pAPC = dynamic_cast<osg::AnimationPathCallback*>(m_pAnimationTransform->getUpdateCallback()))
						{
							bStop = true;
							break;
						}
					}
				}

			}
			// now we have a collision target, check to see if it belongs to a traffic light
			if (TrafficSystem::TrafficLightFacade* pTrafficLightFacade = dynamic_cast<TrafficSystem::TrafficLightFacade*>(pCT))
			{
				// get world position collision point
				osg::Vec3f vTargetPosition = pCT->getFacadeCollisionPoint();

				// now we need to transfer the world position collision traget to the frame of reference (coordinate system)
				// for the collide volume
				vTargetPosition = vTargetPosition * mW2L;

				// finally test to see if the collision target is inside the detector volume
				if (m_pGeode->getBoundingBox().contains(vTargetPosition))
				{
					//std::cout << "Ouch, I have hit a traffic light: " << pTrafficLightFacade->root()->getName() << std::endl;

					if(DemoTrafficLight* pDTL=dynamic_cast<DemoTrafficLight*>(pTrafficLightFacade))
					{
						if (osg::AnimationPathCallback* pAPC = dynamic_cast<osg::AnimationPathCallback*>(m_pAnimationTransform->getUpdateCallback()))
						{
							if (pDTL->getState() == DemoTrafficLight::Stop)
							{
								bStop = true;
								break;
							}
						}
					}
					if (Assignment::ControllableTrafficLightFacade* pDTL = dynamic_cast<Assignment::ControllableTrafficLightFacade*>(pTrafficLightFacade))
					{
						if (osg::AnimationPathCallback* pAPC = dynamic_cast<osg::AnimationPathCallback*>(m_pAnimationTransform->getUpdateCallback()))
						{
							if (pDTL->getState() == Assignment::ControllableTrafficLightFacade::STOP)
							{
								bStop = true;
								break;
							}
						}
					}

				}

			}

		}

	}
	if (bStop) 
	{
		if (osg::AnimationPathCallback* pAPC = dynamic_cast<osg::AnimationPathCallback*>(m_pAnimationTransform->getUpdateCallback()))
		{
			if (pAPC->getPause() == false) pAPC->setPause(true);
		}
	}
	else 
	{
		if (osg::AnimationPathCallback* pAPC = dynamic_cast<osg::AnimationPathCallback*>(m_pAnimationTransform->getUpdateCallback()))
		{
			if (pAPC->getPause() == true) pAPC->setPause(false);
		}
	}
	return false;
}

osg::Vec3f DemoAnimatedCar::getFacadeCollisionPoint()
{
	osg::Vec4f pos(0.0f, 0.0f, 0.0f, 1.0f);

	pos = pos * osg::computeLocalToWorld(m_pCollisionTarget->getParentalNodePaths(m_pRoot)[0]);

	return osg::Vec3f(pos[0], pos[1], pos[2]);
}


void DemoAnimatedCar::setCamera(osg::Camera* pCamera)
{

	m_pCamera = pCamera;
}

osg::Camera* DemoAnimatedCar::camera()
{
	return m_pCamera;
}

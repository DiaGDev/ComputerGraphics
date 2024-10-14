#include <windows.h>
#include <osg/Geode>

#include "DemoAnimatedCar.h"

#include <iostream>

#include <TrafficSystem/TrafficLightFacade.h> 

DemoAnimatedCar::DemoAnimatedCar(std::string sName, osg::Node* pAsset, osg::Matrix m, bool bVisible): TrafficSystem::CarFacade(sName, pAsset, m, bVisible), m_pAnimationTransform(new osg::MatrixTransform()), m_pCollisionTarget(new osg::MatrixTransform())
{
	// insert the animation transform inot the facade sub tree to allow it to control an empty space transform
	// Take note of the order of these operations. This avoids the existing objects being unref() and deleted
	m_pAnimationTransform->addChild(m_pTransformation); 
	m_pRoot->removeChild(m_pTransformation);
	m_pRoot->addChild(m_pAnimationTransform);
	m_pAnimationTransform->setDataVariance(osg::Object::DYNAMIC);


	// set up the position and size of the detector box
	// remeber this is in the coordinate frame of the car

		//toggleVisible(); // uncomment to hide the detector box
	setTransform(osg::Matrixf::translate(1.0f, -2.5f, -0.7f));
	setBound(osg::Vec3f(4.0f, 2.0f, 0.8f));


	// add a collision detector to the bax of the car - we will not be using this, but can be used to stop cars running into each other
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
//	pAPC->setPause(false);
//	pAPC->setAnimationPath(pPath);
	m_pAnimationTransform->setUpdateCallback(pAPC);
}

//this is the callback fundtion - called on each update traversal
bool DemoAnimatedCar::run(osg::Object* object, osg::Object* data)
{
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
					std::cout << "Ouch, I have hit a traffic light: " << pTrafficLightFacade->root()->getName() << std::endl;
				}
			}
		}
	}

	return false;
}

osg::Vec3f DemoAnimatedCar::getFacadeCollisionPoint()
{
	return osg::Vec3f();
}

#pragma once

#include <windows.h>
#include <osg/Vec3f>
#include <list>

namespace TrafficSystem
{
	typedef std::list<class CollisionTarget*> CollisionTargets;

	class CollisionTarget
	{
	public:
		CollisionTarget();
		virtual ~CollisionTarget();

		// this will need re-implmenting to return the collision point in the coordinate system of the root of the facade it exists in 
		virtual osg::Vec3f getFacadeCollisionPoint()=0;

		CollisionTargets& collisionTargets();

	private:
		static CollisionTargets sm_lCollisionTargets;

	};
}

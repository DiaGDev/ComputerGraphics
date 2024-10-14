#pragma once

#include <Common/Facade.h>
#include "CollisionTarget.h"
#include "Collider.h"

namespace TrafficSystem
{
	class CarFacade: public Common::Facade, public Collider, public CollisionTarget
	{
	public:
		CarFacade(std::string sName, osg::Node* pAsset, osg::Matrixf m = osg::Matrixf::identity(), bool bVisible = true);
		virtual ~CarFacade();



		// Inherited via CollisionTarget
		virtual osg::Vec3f getFacadeCollisionPoint() override;

	};
}

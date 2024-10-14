#pragma once
#include <Common/Facade.h>

#include "CollisionTarget.h"

namespace TrafficSystem
{
	class TrafficLightFacade: public Common::Facade, public CollisionTarget
	{
	public:
		TrafficLightFacade(std::string sName, osg::Node* pAsset, osg::Matrixf m = osg::Matrixf::identity(), bool bVisible = true);
		virtual ~TrafficLightFacade();

		void toggleVisible();

		// Inherited via CollisionTarget
		virtual osg::Vec3f getFacadeCollisionPoint() override;


	protected:
		osg::Switch* m_pCollisionTargetSwitch;
		osg::MatrixTransform* m_pCollisionTarget;

		static osg::Geode* sm_pCollisionIndicator;
		bool m_bCTVisible;
	};
}
#include <Windows.h>
#include <osg/Geode>
#include <osg/ShapeDrawable>

#include "TrafficLightFacade.h"

osg::Geode* TrafficSystem::TrafficLightFacade::sm_pCollisionIndicator=0;


TrafficSystem::TrafficLightFacade::TrafficLightFacade(std::string sName, osg::Node* pAsset, osg::Matrixf m, bool bVisible): Facade(sName, pAsset, m, bVisible), m_pCollisionTargetSwitch(new osg::Switch()), m_pCollisionTarget(new osg::MatrixTransform()), m_bCTVisible(true)
{
	m_pCollisionTarget->ref();
	m_pCollisionTargetSwitch->ref();

	m_pCollisionTargetSwitch->addChild(m_pCollisionTarget);
	m_pCollisionTargetSwitch->setAllChildrenOn();

	if(m_pAsset) m_pAsset->getParent(0)->addChild(m_pCollisionTargetSwitch);
	else m_pRoot->addChild(m_pCollisionTargetSwitch);

	if(!sm_pCollisionIndicator)
	{
		sm_pCollisionIndicator = new osg::Geode();
		sm_pCollisionIndicator->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3f(0.0f, 0.0f, 0.0f), 10.0f)));
	}

	m_pCollisionTarget->addChild(sm_pCollisionIndicator);
}

TrafficSystem::TrafficLightFacade::~TrafficLightFacade()
{
	if (m_pCollisionTargetSwitch) m_pCollisionTargetSwitch->unref();
	if (m_pCollisionTarget) m_pCollisionTarget->unref();
}

void TrafficSystem::TrafficLightFacade::toggleVisible()
{
	m_bCTVisible = !m_bCTVisible;
	if (m_bCTVisible) m_pCollisionTargetSwitch->setAllChildrenOn();
	else m_pCollisionTargetSwitch->setAllChildrenOff();
}

osg::Vec3f TrafficSystem::TrafficLightFacade::getFacadeCollisionPoint()
{
	osg::Vec4f pos(0.0f, 0.0f, 0.0f, 1.0f);

	pos = pos *osg::computeLocalToWorld(m_pCollisionTarget->getParentalNodePaths()[0]);

	return osg::Vec3f(pos[0], pos[1], pos[2]);
}

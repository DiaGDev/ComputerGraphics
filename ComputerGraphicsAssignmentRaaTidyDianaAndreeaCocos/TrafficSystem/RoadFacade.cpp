#include <Common/Facade.h>
#include "RoadFacade.h"

#include <Common/NodeFinderT.h>

TrafficSystem::RoadFacade::RoadFacade(std::string sName, osg::Node* pAsset, osg::Matrixf m, bool bVisible): Facade(sName, pAsset, m, bVisible), m_pRotate(new osg::MatrixTransform), m_pScale(new osg::MatrixTransform)
{
	if(m_pAsset && m_pRoot && m_pTransformation)
	{
		m_pRotate->ref();
		m_pScale->ref();

		m_pTransformation->setName("Translation");
		m_pRotate->setName("Rotation");
		m_pScale->setName("Scale");

		// build new transform structure
		m_pTransformation->removeChild(m_pAsset);
		m_pTransformation->addChild(m_pRotate);
		m_pRotate->addChild(m_pScale);
		m_pScale->addChild(m_pAsset);

		// find switches
		Common::NodeFinderT<osg::Switch> idSwitchFinder("IDSwitch");
		Common::NodeFinderT<osg::Switch> animationPointSwitchFinder("AnimationPointSwitch");
		Common::NodeFinderT<osg::Switch> animationIDFinder("AnimationIDSwitch");

		m_pNames = idSwitchFinder.find(m_pAsset);
		m_pAnimationPoints = animationPointSwitchFinder.find(m_pAsset);
		m_pAnimationIDs = animationIDFinder.find(m_pAsset);

		if (m_pNames) m_pNames->ref();
		if (m_pAnimationPoints) m_pAnimationPoints->ref();
		if (m_pAnimationIDs) m_pAnimationIDs->ref();
	}
}

TrafficSystem::RoadFacade::~RoadFacade()
{
	if (m_pRotate) m_pRotate->unref();
	if (m_pScale) m_pScale->unref();
	if (m_pNames) m_pNames->unref();
	if (m_pAnimationPoints) m_pAnimationPoints->unref();
	if (m_pAnimationIDs) m_pAnimationIDs->unref();
}

osg::MatrixTransform* TrafficSystem::RoadFacade::translate()
{
	return m_pTransformation;
}

osg::MatrixTransform* TrafficSystem::RoadFacade::rotate()
{
	return m_pRotate;
}

osg::MatrixTransform* TrafficSystem::RoadFacade::scale()
{
	return m_pScale;
}

void TrafficSystem::RoadFacade::enableNames(bool bEnable) 
{
	if(m_pNames) 
	{
		if (bEnable) m_pNames->setAllChildrenOn();
		else m_pNames->setAllChildrenOff();
	}
}

void TrafficSystem::RoadFacade::enableAnimationPoints(bool bEnable)
{
	if (m_pAnimationPoints)
	{
		if (bEnable) m_pAnimationPoints->setAllChildrenOn();
		else m_pAnimationPoints->setAllChildrenOff();
	}
}

void TrafficSystem::RoadFacade::enableAnimationIDs(bool bEnable)
{
	if (m_pAnimationIDs)
	{
		if (bEnable) m_pAnimationIDs->setAllChildrenOn();
		else m_pAnimationIDs->setAllChildrenOff();
	}
}

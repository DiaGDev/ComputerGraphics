/*#include "DemoTrafficLight.h"

#include "PedestrianRoadTileLights.h"

AnimationViewerTexture::PedestrianRoadTileLights::PedestrianRoadTileLights(std::string sName, osg::Node* pAsset, osg::Matrixf m, bool bVisible) : TrafficSystem::RoadFacade(sName, pAsset, m, bVisible), m_PuiCount(0), m_PitCurrentLight(m_PlLights.end())
{
	pAsset->setUpdateCallback(this);
}

AnimationViewerTexture::PedestrianRoadTileLights::~PedestrianRoadTileLights()
{
}

void AnimationViewerTexture::PedestrianRoadTileLights::addLight(DemoTrafficLight* pDTL)
{
	if (pDTL)
	{
		m_PlLights.push_back(pDTL);

		m_pTransformation->addChild(pDTL->root());

		pDTL->setState(DemoTrafficLight::STOP);
	}
}

bool AnimationViewerTexture::PedestrianRoadTileLights::run(osg::Object* object, osg::Object* data)
{
	DemoTrafficLight* pDTL = dynamic_cast<DemoTrafficLight*>(pSH->facade());
	if (m_PuiCount <= 100)
	{
		if (m_PuiCount == 100)
		{
			m_PuiCount = 0;
			pDTL->toggleState();
		}
		m_PuiCount++;
	}
	return true;
}*/
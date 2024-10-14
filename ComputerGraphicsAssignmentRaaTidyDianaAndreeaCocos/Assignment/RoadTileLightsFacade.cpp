#include "ControllableTrafficLightFacade.h"

#include "RoadTileLightsFacade.h"


Assignment::RoadTileLightsFacade::RoadTileLightsFacade(std::string sName, osg::Node* pAsset, osg::Matrixf m, bool bVisible): TrafficSystem::RoadFacade(sName, pAsset, m, bVisible), m_uiCount(0), m_itCurrentLight(m_lLights.end())
{
	pAsset->setUpdateCallback(this);
}

Assignment::RoadTileLightsFacade::~RoadTileLightsFacade()
{
}

void Assignment::RoadTileLightsFacade::addLight(ControllableTrafficLightFacade* pCTL)
{
	if(pCTL)
	{
		m_lLights.push_back(pCTL);

		m_pTransformation->addChild(pCTL->root());

		pCTL->setState(ControllableTrafficLightFacade::STOP);
	}
}

bool Assignment::RoadTileLightsFacade::run(osg::Object* object, osg::Object* data)
{
	if (m_uiCount <= 100)
	{
		if (m_itCurrentLight == m_lLights.end()) m_itCurrentLight = m_lLights.begin();

		if(m_uiCount==100)
		{
			m_uiCount = 0;

			if (m_itCurrentLight != m_lLights.end()) (*m_itCurrentLight)->setState(ControllableTrafficLightFacade::STOP);

			if (m_lLights.size() > 1)
			{
				m_itCurrentLight++;
				if (m_itCurrentLight == m_lLights.end()) m_itCurrentLight = m_lLights.begin();
			}

			if (m_itCurrentLight != m_lLights.end()) (*m_itCurrentLight)->setState(ControllableTrafficLightFacade::READY);
		}
		else if (m_uiCount == 30) if (m_itCurrentLight != m_lLights.end()) (*m_itCurrentLight)->setState(ControllableTrafficLightFacade::GO);
		else if (m_uiCount == 70) if (m_itCurrentLight != m_lLights.end()) (*m_itCurrentLight)->setState(ControllableTrafficLightFacade::SLOW);

		m_uiCount++;
	}
	return true;
}

#pragma once

#include <TrafficSystem/RoadFacade.h>

namespace TrafficSystem
{
	class TrafficLightFacade;
}

namespace Assignment
{
	class ControllableTrafficLightFacade;

	typedef std::list<ControllableTrafficLightFacade*> Lights;

	class RoadTileLightsFacade: public TrafficSystem::RoadFacade, public osg::Callback
	{
	public:
		RoadTileLightsFacade(std::string sName, osg::Node* pAsset, osg::Matrixf m, bool bVisible);
		virtual ~RoadTileLightsFacade();

		void addLight(ControllableTrafficLightFacade* pCTF);

		virtual bool run(osg::Object* object, osg::Object* data);
	protected:
		Lights m_lLights;

		Lights::iterator m_itCurrentLight;
		unsigned int m_uiCount;
	};
}



/*#pragma once
#include <TrafficSystem/RoadFacade.h>
namespace TrafficSystem
{
	class TrafficLightFacade;
}

namespace AnimationViewerTexture
{
	class DemoTrafficLight;
	typedef std::list<DemoTrafficLight*> PedestrianLights;
	class PedestrianRoadTileLightsFacade : public TrafficSystem::RoadFacade, public osg::Callback
	{
	public:
		PedestrianRoadTileLightsFacade(std::string sName, osg::Node* pAsset, osg::Matrixf m, bool bVisible);
		virtual ~PedestrianRoadTileLightsFacade();

		void addLight(DemoTrafficLight* pDTL);

		virtual bool run(osg::Object* object, osg::Object* data);
	protected:
		PedestrianLights m_PlLights;

		PedestrianLights::iterator m_PitCurrentLight;
		unsigned int m_PuiCount;
	};
}*/

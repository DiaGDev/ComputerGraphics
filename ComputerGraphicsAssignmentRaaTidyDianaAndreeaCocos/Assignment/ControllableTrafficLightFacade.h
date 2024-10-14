#pragma once

#include <windows.h>
#include <osg/Material>

#include <TrafficSystem/TrafficLightFacade.h>

namespace Assignment
{
	class ControllableTrafficLightFacade: public TrafficSystem::TrafficLightFacade
	{
	public:
		enum LightState
		{
			STOP,
			READY,
			GO,
			SLOW,
			ALL,
			NONE,
		};


		ControllableTrafficLightFacade(std::string sname, osg::Node* pAsset, osg::Matrixf m, bool bVisible);
		virtual ~ControllableTrafficLightFacade();
		LightState getState();
		void setState(LightState eState);
	protected:
		static osg::Material* sm_pRedOn;
		static osg::Material* sm_pGreenOn;
		static osg::Material* sm_pAmberOn;
		static osg::Material* sm_pRedOff;
		static osg::Material* sm_pGreenOff;
		static osg::Material* sm_pAmberOff;

		osg::Geode* m_pAmber;
		osg::Geode* m_pRed;
		osg::Geode* m_pGreen;

		LightState trafficState;
	};
}

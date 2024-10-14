#pragma once

#include <windows.h>
#include <osg/Material>

#include <TrafficSystem/TrafficLightFacade.h>

#define WALK_SHADER

class DemoTrafficLight: public TrafficSystem::TrafficLightFacade, public osg::Callback
{
	
public:
	enum TLState
	{
		Stop,
		Go,
	};

	DemoTrafficLight(std::string sName, osg::Node* pAsset, osg::Matrixf m, bool bVisible);
	virtual ~DemoTrafficLight();

	virtual osg::Vec3f getFacadeCollisionPoint() override;

	TLState getState();
	void setState(TLState eState);

	void toggleState();
	unsigned int m_PuiCount=0;

#ifdef WALK_SHADER
	virtual bool run(osg::Object* object, osg::Object* data);
#endif

protected:
	osg::MatrixTransform* m_pSelectTransform;
	TLState m_eState;



private:
	static osg::Material* sm_pStopMaterial;
	static osg::Material* sm_pGoMaterial;

	osg::Geode* m_pRedLamp;
	osg::Geode* m_pAmberLamp;
	osg::Geode* m_pGreenLamp;



#ifdef WALK_SHADER
	osg::Uniform* m_pWalkTimeUniform;
	osg::Uniform* m_pWalkStateUniform;
#else
	osg::Switch* m_pTextureSwitch;
#endif
};


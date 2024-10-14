#include "TrafficLightFacade.h"
#include "TrafficLightFacadeFactory.h"

TrafficSystem::TrafficLightFacadeFactory::TrafficLightFacadeFactory()
{
}

TrafficSystem::TrafficLightFacadeFactory::~TrafficLightFacadeFactory()
{
}

Common::Facade* TrafficSystem::TrafficLightFacadeFactory::create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible)
{
	return new TrafficLightFacade(sName, pModelRoot, rTransform, bVisible);
}

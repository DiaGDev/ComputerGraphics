#include "RoadFacadeFactory.h"

#include "RoadFacade.h"

TrafficSystem::RoadFacadeFactory::RoadFacadeFactory()
{
}

TrafficSystem::RoadFacadeFactory::~RoadFacadeFactory()
{
}

Common::Facade* TrafficSystem::RoadFacadeFactory::create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible)
{
	return new RoadFacade(sName, pModelRoot, rTransform, bVisible);
}

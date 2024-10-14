#include "CarFacadeFactory.h"

#include "CarFacade.h"

Common::Facade* TrafficSystem::CarFacadeFactory::create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible)
{
    return new CarFacade(sName, pModelRoot, rTransform, bVisible);
}

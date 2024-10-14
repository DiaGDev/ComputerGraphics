#include "ControllableTrafficLightFacade.h"
#include "ControllableTrafficLightFacadeFactory.h"

Assignment::ControllableTrafficLightFacadeFactory::ControllableTrafficLightFacadeFactory()
{
}

Assignment::ControllableTrafficLightFacadeFactory::~ControllableTrafficLightFacadeFactory()
{
}

Common::Facade* Assignment::ControllableTrafficLightFacadeFactory::create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible)
{
    return new Assignment::ControllableTrafficLightFacade(sName, pModelRoot, rTransform, bVisible);
}

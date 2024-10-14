#include "RoadTileLightsFacade.h"
#include "RoadTileLightsFacadeFactory.h"

Assignment::RoadTileLightsFacadeFactory::RoadTileLightsFacadeFactory()
{
}

Assignment::RoadTileLightsFacadeFactory::~RoadTileLightsFacadeFactory()
{
}

Common::Facade* Assignment::RoadTileLightsFacadeFactory::create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible)
{
    return new RoadTileLightsFacade(sName, pModelRoot, rTransform, bVisible);
}

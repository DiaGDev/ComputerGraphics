#include "Facade.h"
#include "FacadeFactory.h"

Common::FacadeFactory::FacadeFactory()
{
}

Common::FacadeFactory::~FacadeFactory()
{
}

Common::Facade* Common::FacadeFactory::create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible)
{
    return new Facade(sName, pModelRoot, rTransform, bVisible);
}

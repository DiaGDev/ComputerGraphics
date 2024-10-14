#include "DemoAnimatedCarFactory.h"
#include "DemoAnimatedCar.h"

DemoAnimatedCarFactory::DemoAnimatedCarFactory()
{
}

DemoAnimatedCarFactory::~DemoAnimatedCarFactory()
{
}

Common::Facade* DemoAnimatedCarFactory::create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible)
{
    return new DemoAnimatedCar(sName, pModelRoot, rTransform, bVisible);
}

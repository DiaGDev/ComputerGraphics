#include "DemoTrafficLight.h"
#include "DemoTrafficLightFactory.h"

DemoTrafficLightFactory::DemoTrafficLightFactory()
{
}

DemoTrafficLightFactory::~DemoTrafficLightFactory()
{
}

Common::Facade* DemoTrafficLightFactory::create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible)
{
	return new DemoTrafficLight(sName, pModelRoot, rTransform, bVisible);
}

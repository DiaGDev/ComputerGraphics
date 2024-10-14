#pragma once

#include <Common/FacadeFactoryA.h>

class DemoTrafficLightFactory: public Common::FacadeFactoryA
{
public:
	DemoTrafficLightFactory();
	virtual ~DemoTrafficLightFactory();

	virtual Common::Facade* create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible) override;
};


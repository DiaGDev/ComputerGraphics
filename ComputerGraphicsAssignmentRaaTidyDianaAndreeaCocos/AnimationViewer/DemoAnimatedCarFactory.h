#pragma once

#include <Common/FacadeFactoryA.h>


class DemoAnimatedCarFactory : public Common::FacadeFactoryA
{
public:
	DemoAnimatedCarFactory();
	virtual ~DemoAnimatedCarFactory();


	// Inherited via FacadeFactoryA
	virtual Common::Facade* create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible) override;

};


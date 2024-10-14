#pragma once

#include <Common/FacadeFactory.h>

namespace Assignment
{
	class ControllableTrafficLightFacadeFactory: public Common::FacadeFactoryA
	{
	public:
		ControllableTrafficLightFacadeFactory();
		virtual ~ControllableTrafficLightFacadeFactory();
		virtual Common::Facade* create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible) override;
	};
}

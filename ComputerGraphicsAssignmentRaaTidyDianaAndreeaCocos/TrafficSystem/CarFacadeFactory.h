#pragma once

#include <Common/FacadeFactoryA.h>

namespace TrafficSystem
{
	class CarFacadeFactory: public Common::FacadeFactoryA
	{
		// Inherited via FacadeFactoryA
		virtual Common::Facade* create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible) override;
	};
}

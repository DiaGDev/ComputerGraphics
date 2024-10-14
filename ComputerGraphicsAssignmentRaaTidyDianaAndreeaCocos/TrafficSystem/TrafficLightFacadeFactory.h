#pragma once

#include <Common/FacadeFactoryA.h>

namespace TrafficSystem
{
	class TrafficLightFacadeFactory: public Common::FacadeFactoryA
	{
	public:
		TrafficLightFacadeFactory();
		virtual ~TrafficLightFacadeFactory();

		// Inherited via FacadeFactoryA
		virtual Common::Facade* create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible) override;
	};
}

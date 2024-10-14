#pragma once

#include <Common\FacadeFactoryA.h>

namespace TrafficSystem
{
	class RoadFacadeFactory: public Common::FacadeFactoryA
	{
	public:
		RoadFacadeFactory();
		virtual ~RoadFacadeFactory();

		// Inherited via FacadeFactoryA
		virtual Common::Facade* create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible) override;
	};
}

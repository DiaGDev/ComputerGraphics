#pragma once

#include <Common/FacadeFactoryA.h>

namespace Assignment
{
	class RoadTileLightsFacadeFactory: public Common::FacadeFactoryA
	{
	public:
		RoadTileLightsFacadeFactory();
		virtual ~RoadTileLightsFacadeFactory();

		// Inherited via FacadeFactoryA
		virtual Common::Facade* create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible) override;
	};
}

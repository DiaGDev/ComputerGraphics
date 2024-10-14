#pragma once

#include "FacadeFactoryA.h"

namespace Common
{
	class FacadeFactory: public FacadeFactoryA
	{
	public:
		FacadeFactory();
		virtual ~FacadeFactory();

		virtual Facade* create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform, bool bVisible) override;
	};
}

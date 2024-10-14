#pragma once
#include <windows.h>
#include <osg/Node>

namespace Common
{
	class Facade;

	class FacadeFactoryA
	{
	public:
		virtual Facade* create(std::string sName, osg::Node* pModelRoot, osg::Matrixf rTransform = osg::Matrixf::identity(), bool bVisible = true) = 0;
	};
}

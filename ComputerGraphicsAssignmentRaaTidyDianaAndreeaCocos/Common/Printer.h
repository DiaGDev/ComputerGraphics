#pragma once

#include <windows.h>
#include <osg/NodeVisitor>

namespace Common
{
	class Printer : public osg::NodeVisitor
	{
	public:
		Printer();
		virtual ~Printer();

		virtual void apply(osg::Node& node);

	};
}

#pragma once
#include <windows.h>
#include <string>
#include <osg/NodeVisitor>

namespace Common
{
	class NodeFinder : public osg::NodeVisitor
	{
	public:
		NodeFinder();
		NodeFinder(std::string sName);
		virtual ~NodeFinder();

		osg::Node* find(osg::Node* pNode);

	protected:
		virtual void apply(osg::Node& rNode);

	private:
		std::string m_sNodeName;
		osg::Node* m_pNode;
	};
}

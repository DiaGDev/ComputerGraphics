#include "NodeFinder.h"

Common::NodeFinder::NodeFinder() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), m_sNodeName(""), m_pNode(0)
{
}

Common::NodeFinder::NodeFinder(std::string sName) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), m_sNodeName(sName), m_pNode(0)
{
}

Common::NodeFinder::~NodeFinder()
{
    if (m_pNode) m_pNode->unref();
}

osg::Node* Common::NodeFinder::find(osg::Node* pNode)
{
    if (m_pNode) m_pNode->unref();
    m_pNode = 0;

    traverse(*pNode);

    return nullptr;
}

void Common::NodeFinder::apply(osg::Node& rNode)
{
	if (rNode.getName() == m_sNodeName)
	{
		if (m_pNode) m_pNode->unref();
		m_pNode = &rNode;
		m_pNode->ref();
	}
	traverse(rNode);
}

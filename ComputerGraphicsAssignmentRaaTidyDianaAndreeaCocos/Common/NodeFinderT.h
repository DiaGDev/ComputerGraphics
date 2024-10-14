#pragma once

#include <windows.h>
#include <string>
#include <osg/NodeVisitor>

//NOTE: this is a templated class, so needs to be implemented in the declaration file (*.h)
namespace Common
{
	template<class T>
	class NodeFinderT : public osg::NodeVisitor
	{
	public:
		NodeFinderT(std::string sName);
		virtual ~NodeFinderT();

		virtual void apply(osg::Node& node);

		T* find(osg::Node* pNode);

	private:
		std::string m_sNodeName;
		T* m_pFoundNode;
	};
}

template<class T>
inline Common::NodeFinderT<T>::NodeFinderT(std::string sName) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), m_sNodeName(sName), m_pFoundNode(0)
{
}

template<class T>
inline Common::NodeFinderT<T>::~NodeFinderT()
{
	if (m_pFoundNode) m_pFoundNode->unref();
}

template<class T>
inline void Common::NodeFinderT<T>::apply(osg::Node& node)
{
	if (dynamic_cast<T*>(&node) && node.getName()==m_sNodeName)
	{
		if (m_pFoundNode) m_pFoundNode->unref();
		m_pFoundNode = (T*)&node;
		m_pFoundNode->ref();
	}
	traverse(node);
}

template<class T>
inline T* Common::NodeFinderT<T>::find(osg::Node* pNode)
{
	if (m_pFoundNode) m_pFoundNode->unref();
	m_pFoundNode = 0;

	traverse(*pNode);
	return m_pFoundNode;
}


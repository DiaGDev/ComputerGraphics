#include <windows.h>
#include <osg/Geode>
#include <osg/MatrixTransform>

#include "BoundCalculator.h"


Common::BoundCalculator::BoundCalculator(osg::Node* pNode) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), m_pTarget(0)
{
    m_Transform.makeIdentity();
    traverse(*pNode);
}

Common::BoundCalculator::BoundCalculator(osg::Node* pNode, osg::Geode* pTarget) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), m_pTarget(pTarget)
{
    m_Transform.makeIdentity();
    traverse(*pNode);
}

Common::BoundCalculator::~BoundCalculator()
{
}

void Common::BoundCalculator::apply(osg::Node& node)
{
    traverse(node);
}

void Common::BoundCalculator::apply(osg::Geode& node)
{
	if (m_pTarget && m_pTarget == &node)
	{
		osg::BoundingBox bb, bbt;

		for (unsigned int i = 0; i < node.getNumDrawables(); i++) bb.expandBy(node.getDrawable(i)->getBoundingBox());
		for (unsigned int i = 0; i < 8; i++) bbt.expandBy(bb.corner(i) * m_Transform);

		m_BoundingBox.expandBy(bbt);
	}
	else
	{
		osg::BoundingBox bb, bbt;

		for (unsigned int i = 0; i < node.getNumDrawables(); i++) bb.expandBy(node.getDrawable(i)->getBoundingBox());
		for (unsigned int i = 0; i < 8; i++) bbt.expandBy(bb.corner(i) * m_Transform);

		m_BoundingBox.expandBy(bbt);
	}

	traverse(node);
}

void Common::BoundCalculator::apply(osg::MatrixTransform& node)
{
	m_Transform *= node.getMatrix();
	traverse(node);
}

osg::BoundingBox& Common::BoundCalculator::bound()
{
	return m_BoundingBox;
}

float Common::BoundCalculator::xDim()
{
	return m_BoundingBox.xMax() - m_BoundingBox.xMin();
}

float Common::BoundCalculator::yDim()
{
	return m_BoundingBox.yMax() - m_BoundingBox.yMin();
}

float Common::BoundCalculator::zDim()
{
	return m_BoundingBox.zMax() - m_BoundingBox.zMin();
}

osg::Vec3f Common::BoundCalculator::centre()
{
	return m_BoundingBox.center();
}

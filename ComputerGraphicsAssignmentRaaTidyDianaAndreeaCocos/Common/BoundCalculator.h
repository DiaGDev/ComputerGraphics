#pragma once
#include <windows.h>
#include <osg/NodeVisitor>

namespace Common
{
	class BoundCalculator : osg::NodeVisitor
	{
	public:
		BoundCalculator(osg::Node* pNode);
		BoundCalculator(osg::Node* pNode, osg::Geode* pTarget);
		virtual ~BoundCalculator();
		void apply(osg::Node& node) override;
		void apply(osg::Geode& node) override;
		void apply(osg::MatrixTransform& node) override;
		osg::BoundingBox& bound();
		float xDim();
		float yDim();
		float zDim();
		osg::Vec3f centre();
	protected:
		osg::Matrix m_Transform;
		osg::BoundingBox m_BoundingBox;
		osg::Geode* m_pTarget;
	};
}

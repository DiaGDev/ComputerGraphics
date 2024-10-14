#pragma once
#include <windows.h>
#include <osg/NodeVisitor>

namespace TrafficSystem
{
	class AnimationPointFinder: public osg::NodeVisitor
	{
	public:
		AnimationPointFinder();
		virtual ~AnimationPointFinder();

		bool find(unsigned int uiAnimationPoint, osg::Node* pRoot);

		void print();

		osg::Vec3f positionLocal();
		osg::Vec3f positionToProvidedRoot();

		osg::Quat rotationLocal();
		osg::Quat rotationToProvidedRoot();


	protected:
		osg::MatrixTransform* m_pAnimationPoint;
		osg::Vec3f m_vTranslation;
		osg::Quat m_qRotation;
		osg::Vec3f m_vLocalTranslation;
		osg::Quat m_qLocalRotation;
		unsigned int m_uiName;
		std::string m_sName;

		virtual void apply(osg::MatrixTransform& rMatrixTransform);
		virtual void apply(osg::Node& rNode);


	};
}

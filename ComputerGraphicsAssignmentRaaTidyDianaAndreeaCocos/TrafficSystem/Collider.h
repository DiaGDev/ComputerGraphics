#pragma once

#include <windows.h>
#include <osg/BoundingBox>
#include <osg/NodeCallback>
#include <list>
#include <osg/ShapeDrawable>
#include <osg/Switch>

namespace TrafficSystem
{
	typedef std::list<class Collider*> Colliders;

	class Collider: public osg::NodeCallback
	{
	public:
		Collider(osg::Node* pRoot);
		virtual ~Collider();

		virtual bool run(osg::Object* pObject, osg::Object* pData);

		static void toggleVisible();
		virtual void setTransform(osg::Matrixf mTransform);
		virtual void setBound(osg::Vec3f vBoundDimensions);


	protected:
		osg::Node* m_pCollisionRoot;
		osg::Switch* m_pSwitch;
		osg::MatrixTransform* m_pPosition;
		osg::Geode* m_pGeode;
		osg::Box* m_pShape;
		osg::ShapeDrawable* m_pShapeDrawable;
		static bool sm_bBoundVisble;

		static Colliders sm_lColliders;
	};
}

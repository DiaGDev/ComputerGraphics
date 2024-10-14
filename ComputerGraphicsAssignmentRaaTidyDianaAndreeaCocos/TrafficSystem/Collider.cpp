#include <Windows.h>

#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/Geode>


#include <osg/ShapeDrawable>

#include "CarFacade.h"

TrafficSystem::Colliders TrafficSystem::Collider::sm_lColliders;
bool TrafficSystem::Collider::sm_bBoundVisble = false;

TrafficSystem::Collider::Collider(osg::Node* pRoot): m_pCollisionRoot(pRoot), m_pSwitch(new osg::Switch()), m_pPosition(new osg::MatrixTransform()), m_pShapeDrawable(0), m_pGeode(new osg::Geode())
{
	sm_lColliders.push_back(this);
	if (sm_bBoundVisble) m_pSwitch->setAllChildrenOn();
	else m_pSwitch->setAllChildrenOff();


	m_pCollisionRoot->addUpdateCallback(this);
	m_pSwitch->ref();
	m_pPosition->ref();

	if(dynamic_cast<osg::Group*>(m_pCollisionRoot))	((osg::Group*)m_pCollisionRoot)->addChild(m_pSwitch);
	m_pSwitch->addChild(m_pPosition);


	m_pShapeDrawable = new osg::ShapeDrawable(m_pShape=new osg::Box(osg::Vec3f(-10.0f, -20.0f, 0.0f), 10.0f, 10.0f, 10.0f));
	m_pShapeDrawable->setUseDisplayList(false);
	m_pShape->setDataVariance(osg::Object::DYNAMIC);

	m_pGeode->addDrawable(m_pShapeDrawable);
	m_pPosition->addChild(m_pGeode);
}

TrafficSystem::Collider::~Collider()
{
	sm_lColliders.remove(this);
	if (m_pCollisionRoot) m_pCollisionRoot->removeUpdateCallback(this);
	if (m_pSwitch) m_pSwitch->unref();
	if (m_pPosition) m_pPosition->unref();
}

bool TrafficSystem::Collider::run(osg::Object* pObject, osg::Object* pData)
{
	// note this method will get called once per frame on the update visit.
	// You can use it to perform collision detection and enaction of actions 
	// the process for detecting the CollisionTarget objects and acting on them is outlined below
	// Loop through the collision targets (there is a static list of them in the class)
	//		For each target
	//			get the target position and convert to world coordinates
	//			convert the target world coordinates to local ones for the collider (ie g_pRoot->colider(m_pPosition))
	//			get the bounding box for the local shape (ie the shape drawable for this facarde)
	//			test whether the target is inside or outside the bounding box and act on this

	return false;
}

void TrafficSystem::Collider::toggleVisible()
{
	sm_bBoundVisble = !sm_bBoundVisble;

	for(Colliders::iterator it=sm_lColliders.begin();it!=sm_lColliders.end();it++)
		if (sm_bBoundVisble) (*it)->m_pSwitch->setAllChildrenOn();
		else (*it)->m_pSwitch->setAllChildrenOff();
}

void TrafficSystem::Collider::setTransform(osg::Matrixf mTransform)
{
	m_pPosition->setMatrix(mTransform);
}

void TrafficSystem::Collider::setBound(osg::Vec3f vBoundDimensions)
{
	if (m_pShape)
	{
		m_pGeode->removeDrawable(m_pShapeDrawable);
		if (m_pShapeDrawable) m_pShapeDrawable->unref();

		m_pShapeDrawable = new osg::ShapeDrawable(m_pShape = new osg::Box(osg::Vec3f(0.0f, 0.0f, 0.0f), vBoundDimensions[0], vBoundDimensions[1], vBoundDimensions[2]));
		m_pGeode->addDrawable(m_pShapeDrawable);
	}
}


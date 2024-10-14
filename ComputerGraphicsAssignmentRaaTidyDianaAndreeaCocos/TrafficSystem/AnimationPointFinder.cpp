#include <windows.h>
#include <sstream>
#include <osg/MatrixTransform>
#include "AnimationPointFinder.h"

#include <iostream>

TrafficSystem::AnimationPointFinder::AnimationPointFinder() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), m_pAnimationPoint(0), m_uiName(0), m_sName("")
{
}

TrafficSystem::AnimationPointFinder::~AnimationPointFinder()
{
    if (m_pAnimationPoint) m_pAnimationPoint->unref();
}

bool TrafficSystem::AnimationPointFinder::find(unsigned int uiAnimationPoint, osg::Node* pRoot)
{
    std::stringstream ss;
    ss << uiAnimationPoint;
    m_uiName = uiAnimationPoint;
    m_sName = ss.str();
    if (m_pAnimationPoint) m_pAnimationPoint->unref();

    if (pRoot) traverse(*pRoot);

    return false;
}

void TrafficSystem::AnimationPointFinder::print()
{
    if (m_pAnimationPoint)
    {
        std::cout << "Animation Point [" << m_sName << "] transform from root, pos[" << m_vTranslation[0] << ", " << m_vTranslation[1] << ", " << m_vTranslation[2] << "] rot(quat)[" << m_qRotation[0] << ", " << m_qRotation[1] << ", " << m_qRotation[2] << ", " << m_qRotation[3] << "]" << std::endl;
    }
    else
        std::cout << "No animation pint found" << std::endl;
}

osg::Vec3f TrafficSystem::AnimationPointFinder::positionLocal()
{
    return osg::Vec3f();
}

osg::Vec3f TrafficSystem::AnimationPointFinder::positionToProvidedRoot()
{
    return osg::Vec3f();
}

osg::Quat TrafficSystem::AnimationPointFinder::rotationLocal()
{
    return osg::Quat();
}

osg::Quat TrafficSystem::AnimationPointFinder::rotationToProvidedRoot()
{
    return osg::Quat();
}

void TrafficSystem::AnimationPointFinder::apply(osg::MatrixTransform& rMatrixTransform)
{
    if(rMatrixTransform.getName()==m_sName)
    {
        m_pAnimationPoint = &rMatrixTransform;
        m_pAnimationPoint->ref();
        osg::Vec3 s;
        osg::Quat sr;

        osg::computeLocalToWorld(NodeVisitor::getNodePath()).decompose(m_vTranslation, m_qRotation, s, sr);

        m_pAnimationPoint->getMatrix().decompose(m_vLocalTranslation, m_qLocalRotation, s, sr);
    }
    else
    {
        traverse((osg::Node&)rMatrixTransform);
    }
}

void TrafficSystem::AnimationPointFinder::apply(osg::Node& rNode)
{
    traverse(rNode);
}

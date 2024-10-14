#include "DemoTrafficLight.h"

DemoTrafficLight::DemoTrafficLight(std::string sName, osg::Node* pAsset, osg::Matrixf m, bool bVisible): TrafficLightFacade(sName, pAsset,m, bVisible)
{
    // the collision switch is currently attached to the asset
    // ideally it should be attached to the matrix transform for this facade

    if(m_pCollisionTargetSwitch)
    {
        m_pTransformation->addChild(m_pCollisionTargetSwitch);
        m_pCollisionTargetSwitch->getParent(0)->removeChild(m_pCollisionTargetSwitch);
    }

    // set the position of the collision target - in this case to the local (0,0,0)
    m_pCollisionTarget->setMatrix(osg::Matrixf::identity());
}

DemoTrafficLight::~DemoTrafficLight() 
{
}

osg::Vec3f DemoTrafficLight::getFacadeCollisionPoint()
{
    // currently this is calculating the world position target omn every frame. Ideally, because this is a static object,
    // this position could be calculated in the constructor, stored as a member variable and returned here without repeating the calculation

    osg::Vec3f t, s;
    osg::Quat r, sr;

    // get the path, from the position target to the root, and decompose the resultant matrix to get the world position of the collision target
    osg::computeLocalToWorld(m_pCollisionTarget->getParentalNodePaths(0)[0]).decompose(t, r, s, sr);

    return t;
}

#include "CarFacade.h"

TrafficSystem::CarFacade::CarFacade(std::string sName, osg::Node* pAsset, osg::Matrixf m, bool bVisible): Facade(sName, pAsset, m, bVisible), Collider(m_pTransformation), CollisionTarget()
{
}

TrafficSystem::CarFacade::~CarFacade()
{
}

osg::Vec3f TrafficSystem::CarFacade::getFacadeCollisionPoint()
{
    // if this has a collision point it will need to return the value
    return osg::Vec3f();
}

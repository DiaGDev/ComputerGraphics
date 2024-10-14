#include "CollisionTarget.h"

TrafficSystem::CollisionTargets TrafficSystem::CollisionTarget::sm_lCollisionTargets;


TrafficSystem::CollisionTarget::CollisionTarget()
{
	sm_lCollisionTargets.push_back(this);
}

TrafficSystem::CollisionTarget::~CollisionTarget()
{
	sm_lCollisionTargets.remove(this);
}

TrafficSystem::CollisionTargets& TrafficSystem::CollisionTarget::collisionTargets()
{
	return sm_lCollisionTargets;
}

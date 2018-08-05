/**
 * @file bullet_manager.cpp
 * @brief Tesseract ROS Bullet Manager implementation.
 *
 * @author John Schulman
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 * @copyright Copyright (c) 2013, John Schulman
 *
 * @par License
 * Software License Agreement (BSD-2-Clause)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "tesseract_collision/bullet/bullet_manager.h"

namespace tesseract
{



BulletManager::BulletManager(btCollisionDispatcher* dispatcher) : dispatcher_(dispatcher)
{
}

COWPtr BulletManager::cloneCollisionObject(const std::string& name) const
{
  auto it = m_link2cow.find(name);
  if (it != m_link2cow.end())
    return it->second->clone();

  return nullptr;
}

const COWPtr& BulletManager::getCollisionObject(const std::string& name)
{
  assert(m_link2cow.find(name) != m_link2cow.end());
  return m_link2cow[name];
}

Link2Cow& BulletManager::getCollisionObjects() { return m_link2cow; }

void BulletManager::addCollisionObject(COWPtr& cow)
{
  m_link2cow[cow->getName()] = cow;
}

bool BulletManager::removeCollisionObject(const std::string& name)
{
  auto it = m_link2cow.find(name);
  if (it != m_link2cow.end())
  {
    m_link2cow.erase(name);
    return true;
  }
  return false;
}

bool BulletManager::enableCollisionObject(const std::string& name)
{
  auto it = m_link2cow.find(name);
  if (it != m_link2cow.end())
  {
    it->second->m_enabled = true;
    return true;
  }
  return false;
}

bool BulletManager::disableCollisionObject(const std::string& name)
{
  auto it = m_link2cow.find(name);
  if (it != m_link2cow.end())
  {
    it->second->m_enabled = false;
    return true;
  }
  return false;
}

bool BulletManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Affine3d& pose)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = m_link2cow.find(name);
  if (it != m_link2cow.end())
  {
    it->second->setWorldTransform(convertEigenToBt(pose));
    return true;
  }
  return false;
}

void BulletManager::contactDiscreteTest(const COWPtr& cow, ContactDistanceData& collisions)
{
  CollisionCollector cc(collisions, cow, cow->getContactProcessingThreshold());
  contactTestHelper(cow, collisions, cc);
}

void BulletManager::contactCastTest(const COWPtr& cow, ContactDistanceData& collisions)
{
  CastCollisionCollector cc(collisions, cow, cow->getContactProcessingThreshold());
  contactTestHelper(cow, collisions, cc);
}

void BulletManager::convexSweepTest(const COWPtr& cow, const btTransform& tf1, const btTransform& tf2, ContactDistanceData& collisions)
{
  ROS_ERROR("Convex Sweep Test Currently Not Supported");
//    SweepCollisionCollector cc(collisions, cow);
//    convexSweepTestHelper(cow->getCollisionShape(), tf1, tf2, cc);
}

void BulletManager::contactTestHelper(const COWPtr& cow, ContactDistanceData& collisions, btCollisionWorld::ContactResultCallback &result_callback)
{
  btVector3 aabbMin[2],aabbMax[2];
  cow->getCollisionShape()->getAabb(cow->getWorldTransform(),aabbMin[0],aabbMax[0]);

  //need to increase the aabb for contact thresholds
  btVector3 contactThreshold1(cow->getContactProcessingThreshold(), cow->getContactProcessingThreshold(), cow->getContactProcessingThreshold());
  aabbMin[0] -= contactThreshold1;
  aabbMax[0] += contactThreshold1;

  btCollisionObjectWrapper obA(0, cow->getCollisionShape(), cow.get(), cow->getWorldTransform(), -1, -1);

  for (const auto& lc : m_link2cow)
  {
    assert(!collisions.done);

    const COWPtr cow2 = lc.second;

    cow2->getCollisionShape()->getAabb(cow2->getWorldTransform(),aabbMin[1],aabbMax[1]);

    //need to increase the aabb for contact thresholds
    btVector3 contactThreshold2(cow2->getContactProcessingThreshold(), cow2->getContactProcessingThreshold(), cow2->getContactProcessingThreshold());
    aabbMin[1] -= contactThreshold2;
    aabbMax[1] += contactThreshold2;

    bool aabb_check = (aabbMin[0][0] <= aabbMax[1][0] && aabbMax[0][0] >= aabbMin[1][0]) &&
                      (aabbMin[0][1] <= aabbMax[1][1] && aabbMax[0][1] >= aabbMin[1][1]) &&
                      (aabbMin[0][2] <= aabbMax[1][2] && aabbMax[0][2] >= aabbMin[1][2]);

    if (aabb_check)
    {

      bool needs_collision = needsCollisionCheck(*cow,
                                                 *cow2,
                                                 collisions.req->isContactAllowed,
                                                 false);

      if (needs_collision)
      {
        btCollisionObjectWrapper obB(0, cow2->getCollisionShape(), cow2.get(), cow2->getWorldTransform(), -1, -1);

        btCollisionAlgorithm* algorithm = dispatcher_->findAlgorithm(&obA, &obB, 0, BT_CLOSEST_POINT_ALGORITHMS);
        if (algorithm)
        {
          btBridgedManifoldResult contactPointResult(&obA, &obB, result_callback);
          contactPointResult.m_closestPointDistanceThreshold = result_callback.m_closestDistanceThreshold;
          //discrete collision detection query
          algorithm->processCollision(&obA, &obB, dispatch_info_, &contactPointResult);

          algorithm->~btCollisionAlgorithm();
          dispatcher_->freeCollisionAlgorithm(algorithm);
        }
      }
    }

    if (collisions.done)
      break;
  }
}

void BulletManager::convexSweepTestHelper(const btCollisionShape* shape,
                                          const btTransform& tf1,
                                          const btTransform& tf2,
                                          btCollisionWorld::ConvexResultCallback& cc)
{
//    if (btBroadphaseProxy::isConvex(shape->getShapeType()))
//    {
//      const btConvexShape* convex = static_cast<const btConvexShape*>(shape);
//      m_world->convexSweepTest(convex, tf1, tf2, cc, 0);
//    }
//    else if (btBroadphaseProxy::isCompound(shape->getShapeType()))
//    {
//      const btCompoundShape* compound = static_cast<const btCompoundShape*>(shape);
//      for (int i = 0; i < compound->getNumChildShapes(); ++i)
//      {
//        convexSweepTestHelper(
//            compound->getChildShape(i), tf1 * compound->getChildTransform(i), tf1 * compound->getChildTransform(i), cc);
//      }
//    }
//    else
//    {
//      throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex "
//                               "shapes");
//    }
}

}

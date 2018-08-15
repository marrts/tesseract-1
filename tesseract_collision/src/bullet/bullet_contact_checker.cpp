/**
 * @file bullet_contact_checker.cpp
 * @brief Tesseract ROS Bullet Contact Checker implementation.
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
#include "tesseract_collision/bullet/bullet_contact_checker.h"
#include <tesseract_collision/bullet/bullet_cast_managers.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <iostream>
#include <limits>
#include <octomap/octomap.h>

namespace tesseract
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

BulletContactChecker::BulletContactChecker()
{
  name_ = "BULLET";
  coll_config_ = std::unique_ptr<btDefaultCollisionConfiguration>(new btDefaultCollisionConfiguration());
  dispatcher_ = std::unique_ptr<btCollisionDispatcher>(new btCollisionDispatcher(coll_config_.get()));

  dispatcher_->registerCollisionCreateFunc(
      BOX_SHAPE_PROXYTYPE,
      BOX_SHAPE_PROXYTYPE,
      coll_config_->getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));

  dispatcher_->setDispatcherFlags(dispatcher_->getDispatcherFlags() &
                                  ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);

  manager_.reset(new BulletDiscreteBVHManager(dispatcher_.get()));
}

void BulletContactChecker::calcDistancesDiscrete(ContactResultMap& contacts)
{
  ContactDistanceData collisions(&manager_->getContactRequest(), &contacts);
  manager_->contactTest(collisions);
}

void BulletContactChecker::calcDistancesDiscrete(const ContactRequest& req,
                                                 const TransformMap& transforms,
                                                 ContactResultMap& contacts) const
{
  BulletDiscreteSimpleManager manager(dispatcher_.get());
  ContactDistanceData collisions(&req, &contacts);

  constructDiscreteContactManager<BulletDiscreteSimpleManager>(manager, manager_->getCollisionObjects(), req, transforms);

  manager.contactTest(collisions);
}

void BulletContactChecker::calcDistancesContinuous(const ContactRequest& req,
                                                   const TransformMap& transforms1,
                                                   const TransformMap& transforms2,
                                                   ContactResultMap& contacts) const
{
//  BulletManagerSimple manager(dispatcher_.get());
  BulletCastSimpleManager manager(dispatcher_.get());
  ContactDistanceData collisions(&req, &contacts);

  constructCastContactManager<BulletCastSimpleManager>(manager, manager_->getCollisionObjects(), req, transforms1, transforms2);

  manager.contactTest(collisions);
}

void BulletContactChecker::calcCollisionsDiscrete(ContactResultMap& contacts) { calcDistancesDiscrete(contacts); }
void BulletContactChecker::calcCollisionsDiscrete(const ContactRequest& req,
                                                  const TransformMap& transforms,
                                                  ContactResultMap& contacts) const
{
  calcDistancesDiscrete(req, transforms, contacts);
}

void BulletContactChecker::calcCollisionsContinuous(const ContactRequest& req,
                                                    const TransformMap& transforms1,
                                                    const TransformMap& transforms2,
                                                    ContactResultMap& contacts) const
{
  calcDistancesContinuous(req, transforms1, transforms2, contacts);
}

bool BulletContactChecker::addObject(const std::string& name,
                                     const int& mask_id,
                                     const std::vector<shapes::ShapeConstPtr>& shapes,
                                     const EigenSTL::vector_Affine3d& shape_poses,
                                     const CollisionObjectTypeVector& collision_object_types,
                                     bool enabled)
{
  COWPtr new_cow = createCollisionObject(name, mask_id, shapes, shape_poses, collision_object_types, enabled);
  if (new_cow != nullptr)
  {
    manager_->addCollisionObject(new_cow);
    return true;
  }
  else
  {
    return false;
  }
}

bool BulletContactChecker::removeObject(const std::string& name) { return manager_->removeCollisionObject(name); }
void BulletContactChecker::enableObject(const std::string& name) { manager_->enableCollisionObject(name); }
void BulletContactChecker::disableObject(const std::string& name) { manager_->disableCollisionObject(name); }
void BulletContactChecker::setObjectsTransform(const std::string& name, const Eigen::Affine3d& pose)
{
  manager_->setCollisionObjectsTransform(name, pose);
}

void BulletContactChecker::setObjectsTransform(const std::vector<std::string>& names,
                                               const EigenSTL::vector_Affine3d& poses)
{
  manager_->setCollisionObjectsTransform(names, poses);
}

void BulletContactChecker::setObjectsTransform(const TransformMap& transforms)
{
  manager_->setCollisionObjectsTransform(transforms);
}

void BulletContactChecker::setContactRequest(const ContactRequest& req)
{
  manager_->setContactRequest(req);
}

const ContactRequest& BulletContactChecker::getContactRequest() const { return manager_->getContactRequest(); }

DiscreteContactManagerBasePtr BulletContactChecker::createDiscreteManager(const ContactRequest& req, const TransformMap& transforms) const
{
  BulletDiscreteBVHManagerPtr manager(new BulletDiscreteBVHManager(dispatcher_.get()));

  constructDiscreteContactManager(*manager, manager_->getCollisionObjects(), req, transforms);

  return manager;
}
//void BulletContactChecker::constructBulletObject(BulletDiscreteManagerBase& manager,
//                                                 std::vector<std::string>& active_objects,
//                                                 double contact_distance,
//                                                 const TransformMap& transforms,
//                                                 const std::vector<std::string>& active_links,
//                                                 bool continuous) const
//{
//  for (const auto& transform : transforms)
//  {
//    COWPtr new_cow = manager_->cloneCollisionObject(transform.first);
//    if (!new_cow || !new_cow->m_enabled)
//      continue;

//    assert(new_cow->getCollisionShape());

//    new_cow->setWorldTransform(convertEigenToBt(transform.second));

//    // For descrete checks we can check static to kinematic and kinematic to
//    // kinematic
//    new_cow->m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
//    if (!active_links.empty())
//    {
//      bool check = (std::find_if(active_links.begin(), active_links.end(), [&](std::string link) {
//                      return link == transform.first;
//                    }) == active_links.end());
//      if (check)
//      {
//        new_cow->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
//      }
//    }

//    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
//    {
//      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
//    }
//    else
//    {
//      active_objects.push_back(transform.first);
//      (continuous) ?
//          (new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter) :
//          (new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter);
//    }

//    new_cow->setContactProcessingThreshold(contact_distance);
//    manager.addCollisionObject(new_cow);
//  }
//}

//void BulletContactChecker::constructBulletObject(BulletCastManagerBase& manager,
//                                                 std::vector<std::string>& active_objects,
//                                                 double contact_distance,
//                                                 const TransformMap& transforms1,
//                                                 const TransformMap& transforms2,
//                                                 const std::vector<std::string>& active_links) const
//{
//  assert(transforms1.size() == transforms2.size());

//  auto it1 = transforms1.begin();
//  auto it2 = transforms2.begin();
//  while (it1 != transforms1.end())
//  {
//    COWPtr new_cow = manager_->cloneCollisionObject(it1->first);
//    if (!new_cow || !new_cow->m_enabled)
//    {
//      std::advance(it1, 1);
//      std::advance(it2, 1);
//      continue;
//    }

//    assert(new_cow->getCollisionShape());
//    assert(transforms2.find(it1->first) != transforms2.end());

//    new_cow->m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
//    if (!active_links.empty())
//    {
//      bool check = (std::find_if(active_links.begin(), active_links.end(), [&](std::string link) {
//                      return link == it1->first;
//                    }) == active_links.end());
//      if (check)
//      {
//        new_cow->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
//      }
//    }

//    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
//    {
//      new_cow->setWorldTransform(convertEigenToBt(it1->second));
//      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
//    }
//    else
//    {
//      active_objects.push_back(it1->first);

//      if (btBroadphaseProxy::isConvex(new_cow->getCollisionShape()->getShapeType()))
//      {
//        btConvexShape* convex = static_cast<btConvexShape*>(new_cow->getCollisionShape());
//        assert(convex != NULL);

//        btTransform tf1 = convertEigenToBt(it1->second);
//        btTransform tf2 = convertEigenToBt(it2->second);

//        CastHullShape* shape = new CastHullShape(convex, tf1.inverseTimes(tf2));
//        assert(shape != NULL);

//        new_cow->manage(shape);
//        new_cow->setCollisionShape(shape);
//        new_cow->setWorldTransform(tf1);
//      }
//      else if (btBroadphaseProxy::isCompound(new_cow->getCollisionShape()->getShapeType()))
//      {
//        btCompoundShape* compound = static_cast<btCompoundShape*>(new_cow->getCollisionShape());
//        const Eigen::Affine3d& tf1 = it1->second;
//        const Eigen::Affine3d& tf2 = it2->second;

//        btCompoundShape* new_compound = new btCompoundShape(/*dynamicAABBtree=*/false);

//        for (int i = 0; i < compound->getNumChildShapes(); ++i)
//        {
//          btConvexShape* convex = static_cast<btConvexShape*>(compound->getChildShape(i));
//          assert(convex != NULL);

//          btTransform geomTrans = compound->getChildTransform(i);
//          btTransform child_tf1 = convertEigenToBt(tf1) * geomTrans;
//          btTransform child_tf2 = convertEigenToBt(tf2) * geomTrans;

//          btCollisionShape* subshape = new CastHullShape(convex, child_tf1.inverseTimes(child_tf2));
//          assert(subshape != NULL);

//          if (subshape != NULL)
//          {
//            new_cow->manage(subshape);
//            subshape->setMargin(BULLET_MARGIN);
//            new_compound->addChildShape(geomTrans, subshape);
//          }
//        }

//        new_compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to
//                                                 // have no effect when positive
//                                                 // but has an effect when
//                                                 // negative
//        new_cow->manage(new_compound);
//        new_cow->setCollisionShape(new_compound);
//        new_cow->setWorldTransform(convertEigenToBt(tf1));
//      }
//      else
//      {
//        ROS_ERROR("I can only continuous collision check convex shapes and "
//                  "compound shapes made of convex shapes");
//      }

//      new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter;
//    }

//    new_cow->setContactProcessingThreshold(contact_distance);
//    manager.addCollisionObject(new_cow);
//    std::advance(it1, 1);
//    std::advance(it2, 1);
//  }
//}
}

/**
 * @file fcl_contact_checker.cpp
 * @brief Tesseract ROS FCL contact checker implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (BSD)
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

#include "tesseract_collision/fcl/fcl_contact_checker.h"
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

namespace tesseract
{

FCLContactChecker::FCLContactChecker()
{
  name_ = "FCL";
}

void FCLContactChecker::calcDistancesDiscrete(ContactResultMap& contacts)
{
  ContactDistanceData contact_data(&request_, &contacts);
  manager_.manager_->distance(&contact_data, &distanceCallback);
}

void FCLContactChecker::calcDistancesDiscrete(const ContactRequest& req,
                                              const TransformMap& transforms,
                                              ContactResultMap& contacts) const
{
  FCLManager manager;
  ContactDistanceData contact_data(&req, &contacts);

  std::vector<std::string> active_objects;

  constructFCLObject(manager, active_objects, req.contact_distance, transforms, req.link_names);

  manager.manager_->distance(&contact_data, &distanceCallback);
}

void FCLContactChecker::calcDistancesContinuous(const ContactRequest& req,
                                                const TransformMap& transforms1,
                                                const TransformMap& transforms2,
                                                ContactResultMap& contacts) const
{
  ROS_ERROR("Currently Tesseract FCL implementation does not support continuous distance!");
}

void FCLContactChecker::calcCollisionsDiscrete(ContactResultMap& contacts)
{
  ContactDistanceData contact_data(&request_, &contacts);
  manager_.manager_->collide(&contact_data, &distanceCallback);
}

void FCLContactChecker::calcCollisionsDiscrete(const ContactRequest& req,
                                               const TransformMap& transforms,
                                               ContactResultMap& contacts) const
{
  FCLManager manager;
  ContactDistanceData contact_data(&req, &contacts);

  std::vector<std::string> active_objects;

  constructFCLObject(manager, active_objects, req.contact_distance, transforms, req.link_names);

  manager.manager_->collide(&contact_data, &collisionCallback);
}

void FCLContactChecker::calcCollisionsContinuous(const ContactRequest& req,
                                                 const TransformMap& transforms1,
                                                 const TransformMap& transforms2,
                                                 ContactResultMap& contacts) const
{
  ROS_ERROR("Currently Tesseract FCL implementation does not support continuous collision!");
}

bool FCLContactChecker::addObject(const std::string& name,
                                  const int& mask_id,
                                  const std::vector<shapes::ShapeConstPtr>& shapes,
                                  const EigenSTL::vector_Affine3d& shape_poses,
                                  const CollisionObjectTypeVector& collision_object_types,
                                  bool enabled = true)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty())
  {
    ROS_DEBUG("ignoring link %s", name.c_str());
    return false;
  }
  assert(shapes.size() == shape_poses.size() == CollisionObjectTypeVector.size());
  COWPtr new_cow(new FCLCollisionObjectWrapper(name, mask_id, shapes, shape_poses, collision_object_types));

  if (new_cow)
  {
    new_cow->m_enabled = enabled;
    manager_.addCollisionObject(new_cow);
    ROS_DEBUG("Added collision object for link %s", new_cow->getName().c_str());
    return true;
  }
  else
  {
    ROS_DEBUG("ignoring link %s", name.c_str());
    return false;
  }
}

bool FCLContactChecker::removeObject(const std::string& name) { return manager_.removeCollisionObject(name); }
void FCLContactChecker::enableObject(const std::string& name) { manager_.enableCollisionObject(name); }
void FCLContactChecker::disableObject(const std::string& name) { manager_.disableCollisionObject(name); }
void FCLContactChecker::setObjectsTransform(const std::string& name, const Eigen::Affine3d& pose)
{
  manager_.setCollisionObjectsTransform(name, pose);
}

void FCLContactChecker::setObjectsTransform(const std::vector<std::string>& names, const EigenSTL::vector_Affine3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0u; i < names.size(); ++i)
    setObjectsTransform(names[i], poses[i]);
}

void FCLContactChecker::setObjectsTransform(const TransformMap& transforms)
{
  for (const auto& transform : transforms)
    setObjectsTransform(transform.first, transform.second);
}

void FCLContactChecker::setContactRequest(const ContactRequest& req)
{
  request_ = req;
  active_objects_.clear();

  for (auto& element : manager_.getCollisionObjects())
  {
    // For descrete checks we can check static to kinematic and kinematic to
    // kinematic
    element.second->m_collisionFilterGroup = FCLCollisionFilterGroups::KinematicFilter;
    if (!request_.link_names.empty())
    {
      bool check = (std::find_if(req.link_names.begin(), req.link_names.end(), [&](std::string link) {
                      return link == element.first;
                    }) == req.link_names.end());
      if (check)
      {
        element.second->m_collisionFilterGroup = FCLCollisionFilterGroups::StaticFilter;
      }
    }

    if (element.second->m_collisionFilterGroup == FCLCollisionFilterGroups::StaticFilter)
    {
      element.second->m_collisionFilterMask = FCLCollisionFilterGroups::KinematicFilter;
    }
    else
    {
      active_objects_.push_back(element.first);
      element.second->m_collisionFilterMask = FCLCollisionFilterGroups::StaticFilter | FCLCollisionFilterGroups::KinematicFilter;
    }

    element.second->getBroadphaseHandle()->m_collisionFilterGroup = element.second->m_collisionFilterGroup;
    element.second->getBroadphaseHandle()->m_collisionFilterMask = element.second->m_collisionFilterMask;
  }
}

const ContactRequest& FCLContactChecker::getContactRequest() const { return request_; }

void FCLContactChecker::constructFCLObject(FCLManager& manager,
                                           std::vector<std::string>& active_objects,
                                           double contact_distance,
                                           const TransformMap& transforms,
                                           const std::vector<std::string>& active_links,
                                           bool continuous = false) const
{
  for (const auto& transform : transforms)
  {
    COWPtr new_cow = manager_->cloneCollisionObject(transform.first);
    if (!new_cow || !new_cow->m_enabled)
      continue;

    assert(new_cow->getCollisionShape());

    new_cow->setWorldTransform(convertEigenToBt(transform.second));

    // For descrete checks we can check static to kinematic and kinematic to
    // kinematic
    new_cow->m_collisionFilterGroup = FCLCollisionFilterGroups::KinematicFilter;
    if (!active_links.empty())
    {
      bool check = (std::find_if(active_links.begin(), active_links.end(), [&](std::string link) {
                      return link == transform.first;
                    }) == active_links.end());
      if (check)
      {
        new_cow->m_collisionFilterGroup = FCLCollisionFilterGroups::StaticFilter;
      }
    }

    if (new_cow->m_collisionFilterGroup == FCLCollisionFilterGroups::StaticFilter)
    {
      new_cow->m_collisionFilterMask = FCLCollisionFilterGroups::KinematicFilter;
    }
    else
    {
      active_objects.push_back(transform.first);
      (continuous) ?
          (new_cow->m_collisionFilterMask = FCLCollisionFilterGroups::StaticFilter) :
          (new_cow->m_collisionFilterMask = FCLCollisionFilterGroups::StaticFilter | FCLCollisionFilterGroups::KinematicFilter);
    }

    setContactDistance(new_cow, contact_distance);
    manager.addCollisionObject(new_cow);
  }
}

void FCLContactChecker::constructFCLObject(FCLManager& manager,
                                           std::vector<std::string>& active_objects,
                                           double contact_distance,
                                           const TransformMap& transforms1,
                                           const TransformMap& transforms2,
                                           const std::vector<std::string>& active_links) const
{
  assert(transforms1.size() == transforms2.size());

  auto it1 = transforms1.begin();
  auto it2 = transforms2.begin();
  while (it1 != transforms1.end())
  {
    COWPtr new_cow = manager_->cloneCollisionObject(it1->first);
    if (!new_cow || !new_cow->m_enabled)
    {
      std::advance(it1, 1);
      std::advance(it2, 1);
      continue;
    }

    assert(new_cow->getCollisionShape());
    assert(transforms2.find(it1->first) != transforms2.end());

    new_cow->m_collisionFilterGroup = FCLCollisionFilterGroups::KinematicFilter;
    if (!active_links.empty())
    {
      bool check = (std::find_if(active_links.begin(), active_links.end(), [&](std::string link) {
                      return link == it1->first;
                    }) == active_links.end());
      if (check)
      {
        new_cow->m_collisionFilterGroup = FCLCollisionFilterGroups::StaticFilter;
      }
    }

    if (new_cow->m_collisionFilterGroup == FCLCollisionFilterGroups::StaticFilter)
    {
      new_cow->setWorldTransform(convertEigenToBt(it1->second));
      new_cow->m_collisionFilterMask = FCLCollisionFilterGroups::KinematicFilter;
    }
    else
    {
      active_objects.push_back(it1->first);

      if (btBroadphaseProxy::isConvex(new_cow->getCollisionShape()->getShapeType()))
      {
        btConvexShape* convex = static_cast<btConvexShape*>(new_cow->getCollisionShape());
        assert(convex != NULL);

        btTransform tf1 = convertEigenToBt(it1->second);
        btTransform tf2 = convertEigenToBt(it2->second);

        CastHullShape* shape = new CastHullShape(convex, tf1.inverseTimes(tf2));
        assert(shape != NULL);

        new_cow->manage(shape);
        new_cow->setCollisionShape(shape);
        new_cow->setWorldTransform(tf1);
      }
      else if (btBroadphaseProxy::isCompound(new_cow->getCollisionShape()->getShapeType()))
      {
        btCompoundShape* compound = static_cast<btCompoundShape*>(new_cow->getCollisionShape());
        const Eigen::Affine3d& tf1 = it1->second;
        const Eigen::Affine3d& tf2 = it2->second;

        btCompoundShape* new_compound = new btCompoundShape(/*dynamicAABBtree=*/false);

        for (int i = 0; i < compound->getNumChildShapes(); ++i)
        {
          btConvexShape* convex = static_cast<btConvexShape*>(compound->getChildShape(i));
          assert(convex != NULL);

          btTransform geomTrans = compound->getChildTransform(i);
          btTransform child_tf1 = convertEigenToBt(tf1) * geomTrans;
          btTransform child_tf2 = convertEigenToBt(tf2) * geomTrans;

          btCollisionShape* subshape = new CastHullShape(convex, child_tf1.inverseTimes(child_tf2));
          assert(subshape != NULL);

          if (subshape != NULL)
          {
            new_cow->manage(subshape);
            subshape->setMargin(BULLET_MARGIN);
            new_compound->addChildShape(geomTrans, subshape);
          }
        }

        new_compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to
                                                 // have no effect when positive
                                                 // but has an effect when
                                                 // negative
        new_cow->manage(new_compound);
        new_cow->setCollisionShape(new_compound);
        new_cow->setWorldTransform(convertEigenToBt(tf1));
      }
      else
      {
        ROS_ERROR("I can only continuous collision check convex shapes and "
                  "compound shapes made of convex shapes");
      }

      new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter;
    }

    setContactDistance(new_cow, contact_distance);
    manager.addCollisionObject(new_cow);
    std::advance(it1, 1);
    std::advance(it2, 1);
  }
}

}

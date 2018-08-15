/**
 * @file fcl_utils.h
 * @brief Tesseract ROS FCL Utility Functions.
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

#ifndef TESSERACT_COLLISION_FCL_UTILS_H
#define TESSERACT_COLLISION_FCL_UTILS_H

//#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <memory>
#include <set>
#include <tesseract_core/basic_types.h>
#include <tesseract_collision/contact_checker_common.h>
#include <geometric_shapes/mesh_operations.h>
#include <ros/console.h>

namespace tesseract
{
typedef std::shared_ptr<fcl::CollisionGeometryd> FCLCollisionGeometryPtr;
typedef std::shared_ptr<fcl::CollisionObjectd> FCLCollisionObjectPtr;
typedef std::shared_ptr<const fcl::CollisionObjectd> FCLCollisionObjectConstPtr;

enum FCLCollisionFilterGroups
{
  DefaultFilter = 1,
  StaticFilter = 2,
  KinematicFilter = 4,
  AllFilter = -1 //all bits sets: DefaultFilter | StaticFilter | KinematicFilter
};

class FCLCollisionObjectWrapper
{
public:
  FCLCollisionObjectWrapper(const std::string& name,
                            const int& type_id,
                            const std::vector<shapes::ShapeConstPtr>& shapes,
                            const EigenSTL::vector_Affine3d& shape_poses,
                            const CollisionObjectTypeVector& collision_object_types);

  short int m_collisionFilterGroup;
  short int m_collisionFilterMask;
  bool m_enabled;

  const std::string& getName() const { return name_; }
  const int& getTypeID() const { return type_id_; }

  /** \brief Check if two objects point to the same source object */
  bool sameObject(const FCLCollisionObjectWrapper& other) const
  {
    return name_ == other.name_ && type_id_ == other.type_id_ && &shapes_ == &(other.shapes_) &&
           &shape_poses_ == &(other.shape_poses_);
  }

  void setCollisionObjectsTransform(const Eigen::Affine3d& pose)
  {
    for (unsigned i = 0; i < collision_objects_.size(); ++i)
    {
      FCLCollisionObjectPtr& co = collision_objects_[i];
      Eigen::Affine3d new_t = pose * shape_poses_[i];

      fcl::Transform3d t;
      t.translation() = new_t.translation();
      t.linear() = new_t.rotation();

      co->setTransform(t);
      co->computeAABB();
    }
  }

  const std::vector<FCLCollisionObjectPtr>& getCollisionObjects() const
  {
    return collision_objects_;
  }

  std::vector<FCLCollisionObjectPtr>& getCollisionObjects()
  {
    return collision_objects_;
  }

  std::shared_ptr<FCLCollisionObjectWrapper> clone()
  {
    std::shared_ptr<FCLCollisionObjectWrapper> clone_cow(
        new FCLCollisionObjectWrapper(name_, type_id_, shapes_, shape_poses_, collision_object_types_, collision_geometries_, collision_objects_));
    clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
    clone_cow->m_collisionFilterMask = m_collisionFilterMask;
    clone_cow->m_enabled = m_enabled;
    return clone_cow;
  }

protected:

  FCLCollisionObjectWrapper(const std::string& name,
                            const int& type_id,
                            const std::vector<shapes::ShapeConstPtr>& shapes,
                            const EigenSTL::vector_Affine3d& shape_poses,
                            const CollisionObjectTypeVector& collision_object_types,
                            const std::vector<FCLCollisionGeometryPtr>& collision_geometries,
                            const std::vector<FCLCollisionObjectPtr>& collision_objects);

  std::string name_;  // name of the collision object
  int type_id_;       // user defined type id
  const std::vector<shapes::ShapeConstPtr>& shapes_;
  const EigenSTL::vector_Affine3d& shape_poses_;
  const CollisionObjectTypeVector& collision_object_types_;
  std::vector<FCLCollisionGeometryPtr> collision_geometries_;
  std::vector<FCLCollisionObjectPtr> collision_objects_;
};

FCLCollisionGeometryPtr createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                             const CollisionObjectType& collision_object_type);

typedef FCLCollisionObjectWrapper FCLCOW;
typedef std::shared_ptr<FCLCollisionObjectWrapper> FCLCOWPtr;
typedef std::shared_ptr<const FCLCollisionObjectWrapper> FCLCOWConstPtr;
typedef std::map<std::string, FCLCOWPtr> Link2FCLCOW;
typedef std::map<std::string, FCLCOWConstPtr> Link2ConstFCLCOW;

struct FCLManager
{
  FCLManager()
  {
    manager_ = std::unique_ptr<fcl::BroadPhaseCollisionManagerd>(new fcl::DynamicAABBTreeCollisionManagerd());
  }

  FCLCOWPtr cloneCollisionObject(const std::string& name) const
  {
    auto it = link2cow_.find(name);
    if (it != link2cow_.end())
      return it->second->clone();

    return nullptr;
  }

  const FCLCOWPtr& getCollisionObject(const std::string& name)
  {
    assert(link2cow_.find(name) != link2cow_.end());
    return link2cow_[name];
  }

  Link2FCLCOW& getCollisionObjects() { return link2cow_; }

  void addCollisionObject(FCLCOWPtr& cow)
  {
    link2cow_[cow->getName()] = cow;

    std::vector<FCLCollisionObjectPtr>& objects = cow->getCollisionObjects();
    for (auto& co : objects)
      manager_->registerObject(co.get());
  }

  bool removeCollisionObject(const std::string& name)
  {
    auto it = link2cow_.find(name);
    if (it != link2cow_.end())
    {
      std::vector<FCLCollisionObjectPtr>& objects = it->second->getCollisionObjects();
      for (auto& co : objects)
        manager_->unregisterObject(co.get());

      link2cow_.erase(name);
      return true;
    }
    return false;
  }

  bool enableCollisionObject(const std::string& name)
  {
    auto it = link2cow_.find(name);
    if (it != link2cow_.end())
    {
      it->second->m_enabled = true;
      return true;
    }
    return false;
  }

  bool disableCollisionObject(const std::string& name)
  {
    auto it = link2cow_.find(name);
    if (it != link2cow_.end())
    {
      it->second->m_enabled = false;
      return true;
    }
    return false;
  }

  bool setCollisionObjectsTransform(const std::string& name, const Eigen::Affine3d& pose)
  {
    // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
    // geometry
    auto it = link2cow_.find(name);
    if (it != link2cow_.end())
    {
      it->second->setCollisionObjectsTransform(pose);
      return true;
    }
    return false;
  }

  /// @brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision)
  void collide(void* cdata, fcl::CollisionCallBack<double> callback) const
  {
    manager_->collide(cdata, callback);
  }

  /// @brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance)
  void distance(void* cdata, fcl::DistanceCallBack<double> callback) const
  {
    manager_->distance(cdata, callback);
  }


private:
  std::unique_ptr<fcl::BroadPhaseCollisionManagerd> manager_;
  Link2FCLCOW link2cow_;
};

bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data);

bool distanceCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& min_dist);

inline void transform2fcl(const Eigen::Affine3d& b, fcl::Transform3d& f)
{
  f.translation() = b.translation();
  f.linear() = b.rotation();
}

inline fcl::Transform3d transform2fcl(const Eigen::Affine3d& b)
{
  fcl::Transform3d t;
  transform2fcl(b, t);
  return t;
}

}
#endif // TESSERACT_COLLISION_FCL_UTILS_H

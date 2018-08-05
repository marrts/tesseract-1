/**
 * @file bullet_manager.h
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
#ifndef TESSERACT_COLLISION_BULLET_MANAGER_H
#define TESSERACT_COLLISION_BULLET_MANAGER_H

#include <tesseract_collision/bullet/bullet_utils.h>

namespace tesseract
{

struct BulletManagerBase
{
  /**
   * @brief Make a clone of a collision object
   * @param name The name of the object to clone
   * @return A clone of the object
   */
  virtual COWPtr cloneCollisionObject(const std::string& name) const = 0;

  /**
   * @brief Get a collision object by name
   * @param name The name of the collision object to return
   * @return Collision object
   */
  virtual const COWPtr& getCollisionObject(const std::string& name) = 0;

  /**
   * @brief Get all collision objects managed
   * @return Vector of collision objects
   */
  virtual Link2Cow& getCollisionObjects() = 0;

  /**
   * @brief Add a collision object to the manager
   * @param cow The collision object
   */
  virtual void addCollisionObject(COWPtr& cow) = 0;

  /**
   * @brief Remove a collision object from the manager
   * @param name The name of the collision object
   * @return True if successful, otherwise false
   */
  virtual bool removeCollisionObject(const std::string& name) = 0;

  /**
   * @brief Enable a collision object
   * @param name The name of the collision object
   * @return True if successful, otherwise false
   */
  virtual bool enableCollisionObject(const std::string& name) = 0;

  /**
   * @brief Disable a collision object
   * @param name The name of the collision object
   * @return True if successful, otherwise false
   */
  virtual bool disableCollisionObject(const std::string& name) = 0;

  /**
   * @brief Set a collision objects transformation in world
   * @param name The name of the collision object
   * @param pose The transformation in world
   * @return True if successful, otherwise false
   */
  virtual bool setCollisionObjectsTransform(const std::string& name, const Eigen::Affine3d& pose) = 0;

  virtual void contactDiscreteTest(const COWPtr& cow, ContactDistanceData& collisions) = 0;
  virtual void contactCastTest(const COWPtr& cow, ContactDistanceData& collisions) = 0;
  virtual void convexSweepTest(const COWPtr& cow, const btTransform& tf1, const btTransform& tf2, ContactDistanceData& collisions) = 0;
};
typedef std::shared_ptr<BulletManagerBase> BulletManagerBasePtr;

struct BulletManager : public BulletManagerBase
{
  BulletManager(btCollisionDispatcher* dispatcher);

  COWPtr cloneCollisionObject(const std::string& name) const override;

  const COWPtr& getCollisionObject(const std::string& name) override;

  Link2Cow& getCollisionObjects() override;

  void addCollisionObject(COWPtr& cow) override;

  bool removeCollisionObject(const std::string& name) override;

  bool enableCollisionObject(const std::string& name) override;

  bool disableCollisionObject(const std::string& name) override;

  bool setCollisionObjectsTransform(const std::string& name, const Eigen::Affine3d& pose) override;

  void contactDiscreteTest(const COWPtr& cow, ContactDistanceData& collisions) override;

  void contactCastTest(const COWPtr& cow, ContactDistanceData& collisions) override;

  void convexSweepTest(const COWPtr& cow, const btTransform& tf1, const btTransform& tf2, ContactDistanceData& collisions) override;

private:
  btCollisionDispatcher* dispatcher_;
  btDispatcherInfo dispatch_info_;
  Link2Cow m_link2cow;

  void convexSweepTestHelper(const btCollisionShape* shape,
                             const btTransform& tf1,
                             const btTransform& tf2,
                             btCollisionWorld::ConvexResultCallback& cc);

  void contactTestHelper(const COWPtr& cow,
                         ContactDistanceData& collisions,
                         btCollisionWorld::ContactResultCallback &result_callback);

};
typedef std::shared_ptr<BulletManager> BulletManagerPtr;

}
#endif // TESSERACT_COLLISION_BULLET_MANAGER_H

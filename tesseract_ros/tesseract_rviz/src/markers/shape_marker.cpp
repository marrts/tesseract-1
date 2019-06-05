/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tesseract_rviz/markers/shape_marker.h>
#include <tesseract_rviz/markers/marker_selection_handler.h>
#include <tesseract_rviz/markers/utils.h>

#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>

#include <rviz/ogre_helpers/shape.h>

#include <OgreSceneNode.h>
#include <OgreMatrix3.h>

namespace tesseract_rviz
{

ShapeMarker::ShapeMarker(const std::string &ns,
                         const int id,
                         MarkerType type,
                         rviz::DisplayContext* context,
                         Ogre::SceneNode* parent_node )
  : MarkerBase(ns, id, context, parent_node )
  , shape_(nullptr)
  , type_(type)
  , scale_(Ogre::Vector3(1,1,1))
{
  rviz::Shape::Type shape_type = rviz::Shape::Cube;
  switch( type_ )
  {
  case MarkerType::CUBE:     shape_type = rviz::Shape::Cube;     break;
  case MarkerType::CYLINDER: shape_type = rviz::Shape::Cylinder; break;
  case MarkerType::SPHERE:   shape_type = rviz::Shape::Sphere;   break;
  default:
//    ROS_BREAK();
    break;
  }
  shape_ = new rviz::Shape( shape_type, context_->getSceneManager(), scene_node_ );

  handler_.reset( new MarkerSelectionHandler( this, MarkerID( ns_, id_ ), context_ ));
  handler_->addTrackedObjects( shape_->getRootNode() );
}

ShapeMarker::~ShapeMarker()
{
  delete shape_;
}

void ShapeMarker::setScale(Ogre::Vector3 scale)
{
  scale_ = scale;
  shape_->setScale(scale);
}
Ogre::Vector3 ShapeMarker::getScale() const
{
  return scale_;
}

void ShapeMarker::setColor( float r, float g, float b, float a )
{
  shape_->setColor(r, g, b, a);
}

std::set<Ogre::MaterialPtr> ShapeMarker::getMaterials()
{
  std::set<Ogre::MaterialPtr> materials;
  extractMaterials(shape_->getEntity(), materials);
  return materials;
}

MarkerType ShapeMarker::getType() const {return type_; }

//void ShapeMarker::onNewMessage( const MarkerConstPtr& old_message,
//    const MarkerConstPtr& new_message )
//{
//  if (!shape_ || old_message->type != new_message->type)
//  {
//    delete shape_;
//    shape_ = 0;

//    Shape::Type shape_type = Shape::Cube;
//    switch( new_message->type )
//    {
//    case visualization_msgs::Marker::CUBE:     shape_type = Shape::Cube;     break;
//    case visualization_msgs::Marker::CYLINDER: shape_type = Shape::Cylinder; break;
//    case visualization_msgs::Marker::SPHERE:   shape_type = Shape::Sphere;   break;
//    default:
//      ROS_BREAK();
//      break;
//    }
//    shape_ = new Shape( shape_type, context_->getSceneManager(), scene_node_ );

//    handler_.reset( new MarkerSelectionHandler( this, MarkerID( new_message->ns, new_message->id ), context_ ));
//    handler_->addTrackedObjects( shape_->getRootNode() );
//  }

//  Ogre::Vector3 pos, scale, scale_correct;
//  Ogre::Quaternion orient;
//  transform(new_message, pos, orient, scale);

//  if (owner_ && (new_message->scale.x * new_message->scale.y
//      * new_message->scale.z == 0.0f))
//  {
//    owner_->setMarkerStatus(getID(), StatusProperty::Warn,
//        "Scale of 0 in one of x/y/z");
//  }

//  setPosition(pos);
//  setOrientation( orient * Ogre::Quaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) ) );

//  scale_correct = Ogre::Quaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) ) * scale;

//  shape_->setScale(scale_correct);

//  shape_->setColor(new_message->color.r, new_message->color.g,
//      new_message->color.b, new_message->color.a);
//}

}

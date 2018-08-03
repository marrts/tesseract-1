
#include "tesseract_collision/bullet/bullet_contact_checker.h"
#include "tesseract_collision/fcl/fcl_contact_checker.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

void runTest(tesseract::ContactCheckerBase& checker, bool use_convex_mesh = false)
{
  // Add box to checker  
  shapes::ShapePtr box(new shapes::Box(1, 1, 1));
  Eigen::Affine3d box_pose;
  box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  EigenSTL::vector_Affine3d obj1_poses;
  tesseract::CollisionObjectTypeVector obj1_types;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);
  obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addObject("box_link", 0, obj1_shapes, obj1_poses, obj1_types);

  // Add box to checker
  shapes::ShapePtr thin_box(new shapes::Box(0.1, 1, 1));
  Eigen::Affine3d thin_box_pose;
  thin_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  EigenSTL::vector_Affine3d obj2_poses;
  tesseract::CollisionObjectTypeVector obj2_types;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);
  obj2_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addObject("thin_box_link", 0, obj2_shapes, obj2_poses, obj2_types);

  // Add sphere to checker
  shapes::ShapePtr box_mesh;
  if (use_convex_mesh)
    box_mesh.reset(shapes::createMeshFromResource("package://tesseract_collision/test/box_2m.stl"));
  else
    box_mesh.reset(new shapes::Box(2, 2, 2));

  box_pose.setIdentity();
  Eigen::Affine3d box_mesh_pose;
  box_mesh_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj3_shapes;
  EigenSTL::vector_Affine3d obj3_poses;
  tesseract::CollisionObjectTypeVector obj3_types;
  obj3_shapes.push_back(box_mesh);
  obj3_poses.push_back(box_mesh_pose);
  if (use_convex_mesh)
    obj3_types.push_back(tesseract::CollisionObjectType::ConvexHull);
  else
    obj3_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addObject("box_mesh_link", 0, obj3_shapes, obj3_poses, obj3_types);

  // Check if they are in collision
  tesseract::ContactRequest req;
  req.link_names.push_back("box_link");
  req.link_names.push_back("box_mesh_link");
  req.contact_distance = 0.1;
  req.type = tesseract::ContactRequestType::CLOSEST;

  // Test when object is inside another
  tesseract::TransformMap location;
  location["box_link"] = Eigen::Affine3d::Identity();
  location["box_link"].translation()(0) = 0.2;
  location["box_link"].translation()(1) = 0.1;
  location["box_mesh_link"] = Eigen::Affine3d::Identity();

  tesseract::ContactResultMap result;
  checker.calcDistancesDiscrete(req, location, result);

  tesseract::ContactResultVector result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -1.30, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][0], -0.3, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[1][0], 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], -1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], 0.0, 0.001);


  // Test object is out side the contact distance
  location["box_link"].translation() = Eigen::Vector3d(1.6, 0, 0);
  result.clear();
  result_vector.clear();

  checker.calcCollisionsDiscrete(req, location, result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(result_vector.empty());

  // Test object right at the contact distance
  result.clear();
  result_vector.clear();
  req.contact_distance = 0.25;

  checker.calcDistancesDiscrete(req, location, result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.1, 0.0001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][0], 1.1, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[1][0], 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], -1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], 0.0, 0.001);
}

TEST(TesseractCollisionUnit, FCLCollisionBoxBoxUnit)
{
  tesseract::FCLContactChecker checker;
  runTest(checker);
}

TEST(TesseractCollisionUnit, FCLCollisionBoxBoxConvexHullUnit)
{
  tesseract::FCLContactChecker checker;
  runTest(checker, true);
}

TEST(TesseractCollisionUnit, BulletCollisionBoxBoxUnit)
{
  tesseract::BulletContactChecker checker;
  runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

#ifndef TESSERACT_COLLISION_COLLISION_OCTOMAP_MESH_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_OCTOMAP_MESH_UNIT_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <octomap/octomap.h>
#include <console_bridge/console.h>
#include <tesseract_geometry/mesh_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_collision
{
namespace test_suite
{
namespace detail
{
inline void addCollisionObjects(DiscreteContactManager& checker)
{
  /////////////////////////////////////////////////////////////////
  // Add Octomap
  /////////////////////////////////////////////////////////////////
  std::string path = std::string(TEST_SUITE_DATA_DIR) + "/box_2m.bt";
  auto ot = std::make_shared<octomap::OcTree>(path);
  CollisionShapePtr dense_octomap = std::make_shared<tesseract_geometry::Octree>(ot, tesseract_geometry::Octree::BOX);
  Eigen::Isometry3d octomap_pose;
  octomap_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(dense_octomap);
  obj1_poses.push_back(octomap_pose);

  checker.addCollisionObject("octomap_link", 0, obj1_shapes, obj1_poses);

  /////////////////////////////////////////////////////////////////
  // Add plane mesh to checker.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere = tesseract_geometry::createMeshFromPath<tesseract_geometry::Mesh>(
      std::string(TEST_SUITE_DATA_DIR) + "/plane_4m.stl", Eigen::Vector3d(1, 1, 1), true)[0];

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(sphere);
  obj2_poses.push_back(sphere_pose);

  checker.addCollisionObject("plane_link", 0, obj2_shapes, obj2_poses);
}
}  // namespace detail

inline void runTest(DiscreteContactManager& checker, const std::string& file_path)
{
  // Add collision object
  detail::addCollisionObjects(checker);

  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "octomap_link", "plane_link" });
  checker.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  tesseract_common::TransformMap location;
  location["octomap_link"] = Eigen::Isometry3d::Identity();
  location["plane_link"] = Eigen::Isometry3d::Identity();
  location["plane_link"].translation() = Eigen::Vector3d(0, 0, 0);
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactTestType::ALL);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  const tesseract_collision::CollisionShapesConst& geom = checker.getCollisionObjectGeometries("plane_link");
  const auto& mesh = std::static_pointer_cast<const tesseract_geometry::Mesh>(geom.at(0));
  const auto& mesh_vertices = mesh->getVertices();
  const auto& mesh_triangles = mesh->getTriangles();

  // default color is green
  std::vector<Eigen::Vector3i> mesh_vertices_color(mesh_vertices->size(), Eigen::Vector3i(0, 128, 0));

  for (auto& r : result_vector)
  {
    int idx = 0;
    if (r.link_names[0] != "plane_link")
      idx = 1;

    mesh_vertices_color[static_cast<std::size_t>((*mesh_triangles)[4 * r.subshape_id[idx] + 1])] =
        Eigen::Vector3i(255, 0, 0);
    mesh_vertices_color[static_cast<std::size_t>((*mesh_triangles)[4 * r.subshape_id[idx] + 2])] =
        Eigen::Vector3i(255, 0, 0);
    mesh_vertices_color[static_cast<std::size_t>((*mesh_triangles)[4 * r.subshape_id[idx] + 3])] =
        Eigen::Vector3i(255, 0, 0);
  }

  writeSimplePlyFile(file_path, *mesh_vertices, mesh_vertices_color, *mesh_triangles, mesh->getTriangleCount());

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_TRUE(result_vector.size() == 2712);
}
}  // namespace test_suite
}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_COLLISION_OCTOMAP_MESH_UNIT_HPP

#ifndef TESSERACT_PLANNING_AXIAL_APPROACH_GENERATOR_H
#define TESSERACT_PLANNING_AXIAL_APPROACH_GENERATOR_H

#include <tesseract_process_planning/process_definition.h>
#include <Eigen/Core>

namespace tesseract_process_planning
{

class AxialApproachGenerator : public ProcessStepGenerator
{
public:
  AxialApproachGenerator(const Eigen::Isometry3d& approach, int step_count) : approach_(approach), step_count_(step_count) {}
  ~AxialApproachGenerator() override = default;

  std::vector<tesseract::tesseract_planning::WaypointPtr> generate(const std::vector<tesseract::tesseract_planning::WaypointPtr>& waypoints,
                                                                   const ProcessDefinitionConfig& config) const override
  {
    assert(waypoints.front()->getType() == tesseract::tesseract_planning::WaypointType::CARTESIAN_WAYPOINT);

    std::vector<tesseract::tesseract_planning::WaypointPtr> approach;
    approach.reserve(step_count_ + 1);

    const tesseract::tesseract_planning::CartesianWaypointPtr& cur_waypoint = std::static_pointer_cast<tesseract::tesseract_planning::CartesianWaypoint>(waypoints.front());
    for (int i = step_count_; i >=0; i--)
    {
      tesseract::tesseract_planning::CartesianWaypointPtr new_waypoint = std::make_shared<tesseract::tesseract_planning::CartesianWaypoint>();
      Eigen::Isometry3d scaled = approach_;
      scaled.translation() = (i * 1.0 / step_count_) * approach_.translation();
      new_waypoint->cartesian_position_ = config.world_offset_direction * cur_waypoint->cartesian_position_ * config.local_offset_direction * scaled;
      new_waypoint->coeffs_ = cur_waypoint->coeffs_;
      new_waypoint->is_critical_ = cur_waypoint->is_critical_;
      approach.push_back(new_waypoint);
    }

    return approach;
  }

private:
  Eigen::Isometry3d approach_;
  int step_count_;
};
}

#endif // TESSERACT_PLANNING_AXIAL_APPROACH_GENERATOR_H
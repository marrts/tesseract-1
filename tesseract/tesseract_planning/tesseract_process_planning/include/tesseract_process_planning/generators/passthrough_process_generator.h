#ifndef TESSERACT_PLANNING_PASSTHROUGH_PROCESS_GENERATOR_H
#define TESSERACT_PLANNING_PASSTHROUGH_PROCESS_GENERATOR_H

#include <tesseract_process_planning/process_definition.h>

namespace tesseract_process_planning
{

class PassthroughProcessGenerator : public ProcessStepGenerator
{
public:
  PassthroughProcessGenerator() {}
  ~PassthroughProcessGenerator() = default;

  std::vector<tesseract::tesseract_planning::WaypointPtr> generate(const std::vector<tesseract::tesseract_planning::WaypointPtr>& waypoints,
                                                                   const ProcessDefinitionConfig& config) const override
  {
    std::vector<tesseract::tesseract_planning::WaypointPtr> new_waypoints;
    new_waypoints.reserve(waypoints.size());
    for(const auto& waypoint : waypoints)
    {
      assert(waypoint->getType() == tesseract::tesseract_planning::WaypointType::CARTESIAN_WAYPOINT);
      const tesseract::tesseract_planning::CartesianWaypointPtr& cur_waypoint = std::static_pointer_cast<tesseract::tesseract_planning::CartesianWaypoint>(waypoint);
      tesseract::tesseract_planning::CartesianWaypointPtr new_waypoint = std::make_shared<tesseract::tesseract_planning::CartesianWaypoint>();
      new_waypoint->cartesian_position_ = config.world_offset_direction * cur_waypoint->cartesian_position_ * config.local_offset_direction;
      new_waypoint->coeffs_ = cur_waypoint->coeffs_;
      new_waypoint->is_critical_ = cur_waypoint->is_critical_;
      new_waypoints.push_back(new_waypoint);
    }
    return new_waypoints;
  }
};

}
#endif // TESSERACT_PLANNING_PASSTHROUGH_PROCESS_GENERATOR_H

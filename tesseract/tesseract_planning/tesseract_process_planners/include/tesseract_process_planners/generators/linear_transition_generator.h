#ifndef TESSERACT_PLANNING_LINEAR_TRANSITION_GENERATOR_H
#define TESSERACT_PLANNING_LINEAR_TRANSITION_GENERATOR_H

#include <tesseract_process_planners/process_definition.h>
#include <tesseract_planning/utils.h>
#include <Eigen/Core>

namespace tesseract_process_planners
{

class LinearTransitionGenerator : public ProcessTransitionGenerator
{
public:
  LinearTransitionGenerator(int step_count) : step_count_(step_count) {}
  ~LinearTransitionGenerator() override = default;

  std::vector<tesseract::tesseract_planning::WaypointPtr> generate(const tesseract::tesseract_planning::WaypointPtr& start_waypoint,
                                                                   const tesseract::tesseract_planning::WaypointPtr& end_waypoint) const override
  {
    assert(start_waypoint->getType() == tesseract::tesseract_planning::WaypointType::CARTESIAN_WAYPOINT);
    assert(end_waypoint->getType() == tesseract::tesseract_planning::WaypointType::CARTESIAN_WAYPOINT);

    const tesseract::tesseract_planning::CartesianWaypointPtr& w1 = std::static_pointer_cast<tesseract::tesseract_planning::CartesianWaypoint>(start_waypoint);
    const tesseract::tesseract_planning::CartesianWaypointPtr& w2 = std::static_pointer_cast<tesseract::tesseract_planning::CartesianWaypoint>(end_waypoint);

    return tesseract::tesseract_planning::interpolate(*w1, *w2, step_count_);
  }

private:
  int step_count_;
};

}

#endif // TESSERACT_PLANNING_LINEAR_TRANSITION_GENERATOR_H

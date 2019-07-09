#ifndef TESSERACT_PLANNING_PROCESS_DEFINITION_H
#define TESSERACT_PLANNING_PROCESS_DEFINITION_H

#include <Eigen/Core>
#include <tesseract_planners/core/waypoint.h>

namespace tesseract_process_planning
{

/**
 * @class tesseract_process_planning::ProcessSegmentDefinition
 * @details
 * A process segment definition is assumed to have three components, an approach, process and departure.
 *
 */
struct ProcessSegmentDefinition
{
  /**
   * @brief The approach defined as a series of waypoints.
   *
   * If empty, the approach is skipped
   */
  std::vector<tesseract_planners::WaypointPtr> approach;

  /**
   * @brief The process is defined as a series of waypoints.
   *
   * This should contain a minimum of two waypoints.
   */
  std::vector<tesseract_planners::WaypointPtr> process;

  /**
   * @brief The departure is defined as a series of waypoints.
   *
   * If empty, the approach is skipped
   */
  std::vector<tesseract_planners::WaypointPtr> departure;
};

/**
 * @brief The ProcessTransitionDefinition struct which contains the waypoint data to allow moving between adjacent process segments
 */
struct ProcessTransitionDefinition
{
  std::vector<tesseract_planners::WaypointPtr> transition_from_start; /**< A transition plans from the start of segment[i] to the end of segment[i+1],
   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   this data can be used for finding collision free exit moves after cancelling
   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   an ongoing process */

  std::vector<tesseract_planners::WaypointPtr> transition_from_end;   /**< A transition plans from the end of segment[i] to the start of segment[i+1] */
};

/**
 * @class tesseract_process_planning::ProcessDefinition
 * @details A process definition.
 *
 * This is not intended to support or handle all processes but should covers most of the common ones
 * like sanding, painting, grinding, etc.
 *
 * In a process, we assume they follow this pattern.
 *    * The robot starts from a start position,
 *    * Moves to an approach position just above the object
 *    * Executes a process(sanding, painting, etc.)
 *    * Return to the home position
 *
 * Given the process described the user is only required to define two objects. The start position of
 * the robot and a vector of Process Segment Definitions.
 */
struct ProcessDefinition
{
  tesseract_planners::WaypointPtr start;             /**< The start position of the robot */
  std::vector<ProcessSegmentDefinition> segments;              /**< All of the raster segments with approaches and departures */
  std::vector<ProcessTransitionDefinition> transitions; 	/**< All of the transition to/from a given segment. Must be same length as segments */
};

/**definition
 * @class tesseract_process_planning::ProcessTransitionGenerator
 */
class ProcessTransitionGenerator
{
public:
  virtual ~ProcessTransitionGenerator() = default;
  virtual std::vector<tesseract_planners::WaypointPtr> generate(const tesseract_planners::WaypointPtr& start_waypoint,
                                                                           const tesseract_planners::WaypointPtr& end_waypoint) const = 0;
};
typedef std::shared_ptr<ProcessTransitionGenerator> ProcessTransitionGeneratorPtr;
typedef std::shared_ptr<const ProcessTransitionGenerator> ProcessTransitionGeneratorConstPtr;


/**
 * @class tesseract_process_planning::ProcessDefinitionConfig
 * @details The Process Definition Config
 *
 * This provides the high level process configuration information. It requires the user to provide
 * the home position waypoint (JointWaypoint) and a set of tool paths (strokes) that should be
 * converted into a process results definition leveraging both this configuration information and
 * the ProcessSegmentDefinitions.
 *
 * Also, other operations that are nice to have is the ability to offset the process. Particularly useful
 * when wanting to verify the process without making contact with a surface in the case of sanding.
 */
struct ProcessDefinitionConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  tesseract_planners::WaypointPtr start;
  std::vector<std::vector<tesseract_planners::WaypointPtr>> tool_paths;

  std::vector<ProcessTransitionGeneratorConstPtr> transition_generator;

  Eigen::Isometry3d local_offset_direction;
  Eigen::Isometry3d world_offset_direction;

  ProcessDefinitionConfig()
  {
    local_offset_direction.setIdentity();
    world_offset_direction.setIdentity();
  }
};


/**
 * @brief The ProcessStepGenerator class
 */
class ProcessStepGenerator
{
public:
  virtual ~ProcessStepGenerator() = default;

  virtual std::vector<tesseract_planners::WaypointPtr> generate(const std::vector<tesseract_planners::WaypointPtr>& waypoints,
                                                                           const ProcessDefinitionConfig& config) const = 0;
};
typedef std::shared_ptr<ProcessStepGenerator> ProcessStepGeneratorPtr;
typedef std::shared_ptr<const ProcessStepGenerator> ProcessStepGeneratorConstPtr;


/**
 * @brief The Process Segment Definition Configuration
 *
 * In most manufacturing process like (sanding, painting, etc.) it includes a series of process paths (strokes).
 * Each of the process paths may have an approach, departure and transition step.
 *
 * Example: In the case of painting, when approaching the part you may want to take a specific trajectory
 * and the same for when leaving the part.
 */
struct ProcessSegmentDefinitionConfig
{
  ProcessStepGeneratorConstPtr approach;
  ProcessStepGeneratorConstPtr process;
  ProcessStepGeneratorConstPtr departure;
};


ProcessDefinition generateProcessDefinition(const ProcessDefinitionConfig& process_config,
                                            const ProcessSegmentDefinitionConfig& segment_config);

ProcessDefinition generateProcessDefinition(const ProcessDefinitionConfig& process_config,
                                            const std::vector<ProcessSegmentDefinitionConfig>& segment_config);


}

#endif //TESSERACT_PLANNING_PROCESS_DEFINITION_H


#include <tesseract_process_planning/process_definition.h>

namespace tesseract_process_planning
{

ProcessDefinition generateProcessDefinition(const ProcessDefinitionConfig& process_config, const ProcessSegmentDefinitionConfig& segment_config)
{
  ProcessDefinition process_definition;
  process_definition.start = process_config.start;
  process_definition.segments.reserve(process_config.tool_paths.size());
  process_definition.transitions.reserve(process_config.tool_paths.size() - 1);

  for(size_t i = 0; i < process_config.tool_paths.size(); ++i)
  {
    ProcessSegmentDefinition segment_def;
    if (segment_config.approach != nullptr)
      segment_def.approach = segment_config.approach->generate(process_config.tool_paths[i], process_config);

    if (segment_config.process != nullptr)
      segment_def.process = segment_config.process->generate(process_config.tool_paths[i], process_config);

    if (segment_config.departure != nullptr)
      segment_def.departure = segment_config.departure->generate(process_config.tool_paths[i], process_config);

    process_definition.segments.push_back(segment_def);
  }

  for(size_t i = 0; i < (process_config.tool_paths.size() - 1); ++i)
  {
    ProcessTransitionDefinition transition_def;
    if (process_config.transition_generator[i] != nullptr)
    {
      transition_def.transition_from_end = process_config.transition_generator[i]->generate(process_definition.segments[i].departure.back(), process_definition.segments[i + 1].approach.front());
      transition_def.transition_from_start = process_config.transition_generator[i]->generate(process_definition.segments[i].approach.front(), process_definition.segments[i + 1].departure.back());
    }

    process_definition.transitions.push_back(transition_def);
  }

  return process_definition;
}

}

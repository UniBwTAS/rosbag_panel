#ifndef FILTERED_BAG_GENERATION_HPP
#define FILTERED_BAG_GENERATION_HPP

#include <ros/ros.h>
#include <rosbag/bag.h>

using namespace rosbag;

class FilteredBagGeneration
{
  public:
    static void filter(const std::vector<std::string>& bag_filenames,
                       double relative_start,
                       double duration,
                       const std::vector<std::string>& topics,
                       const std::string& bag_output_filename);
};

#endif

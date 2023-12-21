#include <rosbag/message_instance.h>
#include <rosbag/view.h>

#include "filtered_bag_generation.h"

bool isLatching(const ConnectionInfo* c)
{
    ros::M_string::const_iterator header_iter = c->header->find("latching");
    return (header_iter != c->header->end() && header_iter->second == "1");
}

void FilteredBagGeneration::filter(const std::vector<std::string>& bag_filenames,
                                   double relative_start,
                                   double duration,
                                   const std::vector<std::string>& topics,
                                   const std::string& bag_output_filename)
{
    // Open all the input bag files
    std::vector<std::shared_ptr<Bag>> intput_bags;
    for (std::string const& filename : bag_filenames)
    {
        ROS_INFO_STREAM("Opening input bag: " << filename);
        try
        {
            auto bag = std::make_shared<Bag>();
            bag->open(filename, bagmode::Read);
            intput_bags.push_back(bag);
        }
        catch (const BagUnindexedException& ex)
        {
            ROS_ERROR_STREAM("Bag file " << filename << " is unindexed.  Run rosbag reindex.");
            return;
        }
    }

    // get absolute times required for bag filtering
    View full_view;
    for (const auto& bag : intput_bags)
        full_view.addQuery(*bag);
    ros::Time full_absolute_start_time = full_view.getBeginTime();
    ros::Time desired_absolute_start_time = full_absolute_start_time + ros::Duration(relative_start);
    ros::Time desired_absolute_end_time = desired_absolute_start_time + ros::Duration(duration);

    // Open output bag file
    auto output_bag = std::make_shared<Bag>();
    ROS_INFO_STREAM("Opening output bag: " << bag_output_filename);
    try
    {
        output_bag->open(bag_output_filename, bagmode::Write);
    }
    catch (const rosbag::BagException& e)
    {
        ROS_ERROR("Error writing: %s", e.what());
        return;
    }

    // get view of desired range
    View cropped_view;
    if (topics.empty())
        for (const auto& bag : intput_bags)
            cropped_view.addQuery(*bag, desired_absolute_start_time, desired_absolute_end_time);
    else
        for (const auto& bag : intput_bags)
            cropped_view.addQuery(*bag, TopicQuery(topics), desired_absolute_start_time, desired_absolute_end_time);
    if (cropped_view.size() == 0)
    {
        ROS_WARN_STREAM("No messages on specified topics and range. Nothing to do.");
        return;
    }

    // find last message from latch topics (for each caller id) in the truncated front part
    if (relative_start > 0.0)
    {
        // Retrieve all the latch topics before the desired start
        ROS_INFO_STREAM("Find and write latched messages from truncated front of bag file");
        View truncated_start_view;
        if (topics.empty())
            for (const auto& bag : intput_bags)
                truncated_start_view.addQuery(*bag, full_absolute_start_time, desired_absolute_start_time);
        else
            for (const auto& bag : intput_bags)
                truncated_start_view.addQuery(
                    *bag, TopicQuery(topics), full_absolute_start_time, desired_absolute_start_time);

        // find all caller ids per latched topic
        std::set<std::pair<std::string, std::string>> latch_topics;
        for (const auto& c : truncated_start_view.getConnections())
        {
            if (isLatching(c))
            {
                const auto header_iter = c->header->find("callerid");
                const auto caller_id = (header_iter != c->header->end() ? header_iter->second : std::string(""));
                latch_topics.emplace(caller_id, c->topic);
            }
        }

        // write the last message of each latch topic per caller id:
        for (const auto& item : latch_topics)
        {
            const auto& caller_id = item.first;
            const auto& topic = item.second;

            View latch_view;
            for (const auto& bag : intput_bags)
                latch_view.addQuery(*bag, TopicQuery(topic), full_absolute_start_time, desired_absolute_start_time);

            auto last_message = latch_view.end();
            for (auto iter = latch_view.begin(); iter != latch_view.end(); ++iter)
                if (iter->getCallerId() == caller_id)
                    last_message = iter;

            if (last_message != latch_view.end())
                output_bag->write(
                    topic, desired_absolute_start_time, *last_message, last_message->getConnectionHeader());
        }
    }

    // write cropped view (and visualize nice progress bar)
    int bar_width = 70;
    uint64_t written_messages = 0;
    for (const MessageInstance& m : cropped_view)
    {
        output_bag->write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
        written_messages++;

        // the following is just a nice progress bar visualization
        if (written_messages % 100 == 0)
        {
            double percentage = static_cast<double>(written_messages) / cropped_view.size();

            std::cout << "[";
            int pos = static_cast<int>(std::round(bar_width * percentage));
            for (int i = 0; i < bar_width; ++i)
            {
                if (i < pos)
                    std::cout << "=";
                else if (i == pos)
                    std::cout << ">";
                else
                    std::cout << " ";
            }
            std::cout << "] " << static_cast<int>(std::round(percentage * 100)) << " %\r";
            std::cout.flush();
        }
    }
    std::cout << std::endl;

    // properly close output bag
    output_bag->close();
    ROS_INFO_STREAM("Finished! Output file: " << bag_output_filename);
}
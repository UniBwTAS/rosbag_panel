#include "rviz/display_context.h"
#include "rviz/tool_manager.h"
#include <std_srvs/SetBool.h>

#include "rosbag_tool.h"

namespace rosbag_tool
{

RosbagTool::RosbagTool()
{
    shortcut_key_ = ' ';

    service_client_ = nh_.serviceClient<std_srvs::SetBool>("/rosbag_play/pause_playback");
}

RosbagTool::~RosbagTool()
{
}

void RosbagTool::onInitialize()
{
}

void RosbagTool::activate()
{
    std_srvs::SetBool srv;

    // Try play
    srv.request.data = false;
    service_client_.call(srv);

    // Otherwise pause
    if (!srv.response.success) {
        srv.request.data = true;
        service_client_.call(srv);
    }

    rviz::ToolManager* tool_manager = context_->getToolManager();
    tool_manager->setCurrentTool(tool_manager->getDefaultTool());
}

void RosbagTool::deactivate()
{
}

} // end namespace rosbag_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rosbag_tool::RosbagTool, rviz::Tool)
// END_TUTORIAL
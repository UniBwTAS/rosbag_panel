#pragma once

#include <QKeyEvent>
#include <rviz/render_panel.h>
#include <rviz/tool.h>
#include <ros/ros.h>

namespace rviz
{
class DisplayContext;
}

namespace rosbag_tool
{

class RosbagTool : public rviz::Tool
{
    Q_OBJECT
  public:
    RosbagTool();
    ~RosbagTool();

    virtual void onInitialize();

    virtual void activate();
    virtual void deactivate();

  private:
    ros::ServiceClient service_client_;
    ros::NodeHandle nh_;
};

} // end namespace rtdb_stream_tool

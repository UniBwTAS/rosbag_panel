#pragma once

#include <QDialog>
#include <QLabel>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class QImageDialog : public QDialog
{
  public:
    QImageDialog(QWidget* parent, const ros::NodeHandle& nh);

    void setTopic(const std::string& topic, bool compressed = false);

  private:
    template<typename T> void imageCallback(const T& msg);

  private:
    ros::Subscriber sub_;
    ros::NodeHandle nh_;

    cv::Mat conversion_mat_;

    QLabel* label_;
};

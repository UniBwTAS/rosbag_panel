#pragma once

#ifndef Q_MOC_RUN
#include <thread>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Time.h>
#include <topic_tools/shape_shifter.h>

#include <QProxyStyle>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QTreeView>
#include <rosbag/BagInfo.h>
#include <rosbag/OpenBags.h>
#include <rosbag/PlayOptions.h>
#include <rosgraph_msgs/Clock.h>
#include <rviz/panel.h>

#include <ui_options_dialog.h>

#include "image_dialog.h"
#include "range_slider.h"
#endif

namespace rosbag_panel
{

class DirectJumpProxyStyle : public QProxyStyle
{
  public:
    using QProxyStyle::QProxyStyle;

    int styleHint(QStyle::StyleHint hint,
                  const QStyleOption* option = nullptr,
                  const QWidget* widget = nullptr,
                  QStyleHintReturn* returnData = nullptr) const
    {
        if (hint == QStyle::SH_Slider_AbsoluteSetButtons)
            return (Qt::LeftButton | Qt::MidButton | Qt::RightButton);
        return QProxyStyle::styleHint(hint, option, widget, returnData);
    }
};

class RosbagPanel : public rviz::Panel
{
    Q_OBJECT
  public:
    explicit RosbagPanel(QWidget* parent = nullptr);
    ~RosbagPanel() override;
    void save(rviz::Config config) const override;
    void load(const rviz::Config& config) override;

  public:
    static const int RES = 1000000;

  protected Q_SLOTS:
    void togglePlayPause() const;
    void openFile();
    void openTopicDialog(const std::string& topics_type, std::vector<std::string>& checked_topics);
    void openOptionsDialog();
    void setSpeed(double speed);
    void sliderPressed();
    void timeSliderMoved();
    void rangeSliderMoved();
    void timeSliderReleased();
    void rangeSliderReleased();
    void checkServiceAvailability();
    void toggleExpandWidget();
    void checkIfUsePauseTopics(bool send = true);
    void generateFilteredBag();

  protected:
    QPushButton* play_pause_button_;
    QPushButton* open_button_;
    QPushButton* options_button_;
    QToolButton* expand_button_;
    QSlider* time_slider_;
    QRangeSlider* range_slider_;
    QDoubleSpinBox* speed_spin_box_;
    QLabel* status_label_;

    QWidget* expanded_widget_;
    QCheckBox* use_pause_topic_check_box_;
    QPushButton* generate_filtered_bag_button_;

    ros::NodeHandle nh_;

  private:
    void callbackBagInfo(const rosbag::BagInfoConstPtr& msg);
    void callbackClock(const rosgraph_msgs::ClockConstPtr& msg);

    void resetGui();
    void disableInputs(bool disable);
    QList<QStringList> generateTopicInfos(const std::string& topics_type);
    static QStringList findTopicInfo(const std::string& topic, QList<QStringList> topic_infos);
    void saveConfig() const;
    void loadConfig();
    static std::string join(const std::vector<std::string>& v);
    static std::vector<std::string> split(const std::string& s);

    void startTfFromXmlChildProcess();
    void stopTfFromXmlChildProcess();

    ros::WallTime last_slider_update_{0, 0};

    rosbag::BagInfoConstPtr last_bag_info_;
    ros::Time start_time_;
    ros::Time end_time_;
    ros::Time prev_time_;

    // rosbag open
    rosbag::OpenBags open_bags_srv_;

    // rosbag options
    rosbag::PlayOptions play_options_srv_;

    // plugin options
    std::string topic_image_preview_;
    bool overwrite_tf_static_from_xml_{false};
    bool hide_lower_slider_{false};

    std::vector<std::string> included_pause_topics_;
    std::vector<std::string> advertised_pause_topics_;

    ros::Subscriber bag_info_sub_;
    ros::Subscriber clock_sub_;

    ros::Publisher query_options_pub_;

    QImageDialog* image_dialog_;

    std::string bag_folder_;
    std::string config_path_;

    pid_t pid_pub_tf_static_from_xml_{0};

  private:
    static float getSliderValue(QSlider* slider)
    {
        return static_cast<float>(static_cast<double>(slider->value()) / RES);
    }

    static void setSliderValue(QSlider* slider, double value)
    {
        slider->setValue(static_cast<int>(std::round(value * RES)));
    }

    static float getSliderValue(QRangeSlider* slider, QRangeSlider::HandleType handle)
    {
        return static_cast<float>(static_cast<double>(slider->value(handle)) / RES);
    }

    static void setSliderValue(QRangeSlider* slider, QRangeSlider::HandleType handle, double value)
    {
        slider->setValue(handle, static_cast<int>(std::round(value * RES)));
    }
};

} // end namespace rosbag_panel

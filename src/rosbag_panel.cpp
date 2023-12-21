#include <fstream>
#include <sys/prctl.h>
#include <sys/wait.h>

#include <QFileDialog>
#include <QTimer>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>

#include <rosbag/MessageQuery.h>
#include <rosbag/QueryOptions.h>

#include "filtered_bag_generation.h"
#include "rosbag_panel.h"
#include "std_srvs/SetBool.h"
#include "treemodel.h"

namespace rosbag_panel
{

RosbagPanel::RosbagPanel(QWidget* parent)
    : rviz::Panel(parent),
      expand_button_(new QToolButton()),
      time_slider_(new QSlider(Qt::Horizontal)),
      range_slider_(new QRangeSlider()),
      speed_spin_box_(new QDoubleSpinBox()),
      status_label_(new QLabel("Status: no bags opened")),
      image_dialog_(new QImageDialog(this, nh_))
{
    auto* layout_obj = new QGridLayout(this);

    time_slider_->setStyle(new DirectJumpProxyStyle(time_slider_->style()));
    time_slider_->hide();

    open_button_ = new QPushButton(QIcon::fromTheme("document-open"), "  Open");
    play_pause_button_ = new QPushButton(QIcon::fromTheme("media-playback-start"), "  Play");
    options_button_ = new QPushButton(QIcon::fromTheme("xfce-system-settings"), "  Options");

    expand_button_->setArrowType(Qt::ArrowType::RightArrow);
    expand_button_->setCheckable(true);
    expand_button_->setChecked(false);
    expanded_widget_ = new QWidget();
    use_pause_topic_check_box_ = new QCheckBox("Use pause topics");
    use_pause_topic_check_box_->setChecked(false);
    generate_filtered_bag_button_ = new QPushButton("Generate Bag");
    auto* expanded_layout_obj = new QBoxLayout(QBoxLayout::Direction::LeftToRight, expanded_widget_);
    expanded_layout_obj->addWidget(use_pause_topic_check_box_);
    expanded_layout_obj->addWidget(generate_filtered_bag_button_);
    expanded_widget_->hide();

    auto* horizontal_line = new QFrame();
    horizontal_line->setFrameShape(QFrame::HLine);
    horizontal_line->setFrameShadow(QFrame::Sunken);

    layout_obj->addWidget(open_button_, 0, 0);
    layout_obj->addWidget(speed_spin_box_, 0, 1);
    layout_obj->addWidget(play_pause_button_, 0, 2);
    layout_obj->addWidget(options_button_, 0, 3);
    layout_obj->addWidget(expand_button_, 0, 4);
    layout_obj->addWidget(expanded_widget_, 1, 0, 1, -1);
    layout_obj->addWidget(range_slider_, 2, 0, 1, -1);
    layout_obj->addWidget(time_slider_, 3, 0, 1, -1);
    layout_obj->addWidget(horizontal_line, 4, 0, 1, -1);
    layout_obj->addWidget(status_label_, 5, 0, 1, -1);

    connect(play_pause_button_, SIGNAL(clicked()), this, SLOT(togglePlayPause()));
    connect(open_button_, SIGNAL(clicked()), this, SLOT(openFile()));
    connect(options_button_, SIGNAL(clicked()), this, SLOT(openOptionsDialog()));
    connect(expand_button_, SIGNAL(clicked()), this, SLOT(toggleExpandWidget()));
    connect(time_slider_, SIGNAL(sliderPressed()), this, SLOT(sliderPressed()));
    connect(time_slider_, SIGNAL(sliderMoved(int)), this, SLOT(timeSliderMoved()));
    connect(time_slider_, SIGNAL(sliderReleased()), this, SLOT(timeSliderReleased()));
    connect(range_slider_, SIGNAL(sliderPressed()), this, SLOT(sliderPressed()));
    connect(range_slider_, SIGNAL(sliderMoved(int)), this, SLOT(rangeSliderMoved()));
    connect(range_slider_, SIGNAL(sliderReleased()), this, SLOT(rangeSliderReleased()));
    connect(speed_spin_box_, SIGNAL(valueChanged(double)), this, SLOT(setSpeed(double)));
    connect(use_pause_topic_check_box_, SIGNAL(stateChanged(int)), this, SLOT(checkIfUsePauseTopics()));
    connect(generate_filtered_bag_button_, SIGNAL(clicked()), this, SLOT(generateFilteredBag()));

    bag_info_sub_ = nh_.subscribe("rosbag_play/bag_info", 1, &RosbagPanel::callbackBagInfo, this);
    clock_sub_ = nh_.subscribe("/clock", 1, &RosbagPanel::callbackClock, this);

    auto* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(checkServiceAvailability()));
    timer->start(500);

    resetGui();

    if (!nh_.param<bool>("/use_sim_time", false))
    {
        ROS_WARN("ROS Parameter '/use_sim_time' not set to 'true'. Set it to 'true' in order to use rosbag play panel "
                 "in rviz.");
        status_label_->setText("Status: Warning, ROS Parameter '/use_sim_time' not set to 'true'.");
    }

    query_options_pub_ = nh_.advertise<rosbag::MessageQuery>("rosbag_play/message_query", 1);
}

RosbagPanel::~RosbagPanel()
{
    saveConfig();
}

void RosbagPanel::togglePlayPause() const
{
    std_srvs::SetBool srv;
    srv.request.data = !play_options_srv_.request.pause;
    ros::service::call("/rosbag_play/pause_playback", srv);
}

void RosbagPanel::openFile()
{
    QString dir = "";
    if (!open_bags_srv_.request.bag_files.empty())
        dir = QString::fromStdString(open_bags_srv_.request.bag_files[0]);
    QStringList bag_files = QFileDialog::getOpenFileNames(this, tr("Open Rosbags"), dir, "Rosbags (*.bag)");
    if (!bag_files.isEmpty())
    {
        disableInputs(true);

        std::vector<std::string> bags_paths;
        for (const auto& bag_file : bag_files)
            bags_paths.push_back(bag_file.toStdString());
        open_bags_srv_.request.bag_files = bags_paths;

        // save config to previous bag file first
        saveConfig();

        status_label_->setText("Status: Opening *.bag files.");
        ros::service::call("/rosbag_play/open_bags", open_bags_srv_);
    }
}

void RosbagPanel::setSpeed(double speed)
{
    play_options_srv_.request.offset = play_options_srv_.request.duration * getSliderValue(time_slider_);
    play_options_srv_.request.rate = static_cast<float>(speed);

    // only send service on manual input
    if (!speed_spin_box_->isEnabled())
        return;

    ros::service::call("/rosbag_play/play_options", play_options_srv_);
}

void RosbagPanel::sliderPressed()
{
    last_slider_update_ = ros::WallTime(0, 0);

    if (topic_image_preview_.length() > 0)
        image_dialog_->show();
}

void RosbagPanel::timeSliderMoved()
{
    rosbag::MessageQuery msg;
    msg.offset = play_options_srv_.request.start + play_options_srv_.request.duration * getSliderValue(time_slider_);
    msg.range = 1;
    query_options_pub_.publish(msg);
}

void RosbagPanel::rangeSliderMoved()
{
    float bag_duration = static_cast<float>((end_time_ - start_time_).toSec());

    rosbag::MessageQuery msg;
    msg.offset = bag_duration * getSliderValue(range_slider_);
    msg.range = 1;
    query_options_pub_.publish(msg);
}

void RosbagPanel::timeSliderReleased()
{
    play_options_srv_.request.offset = play_options_srv_.request.duration * getSliderValue(time_slider_);
    ros::service::call("/rosbag_play/play_options", play_options_srv_);

    last_slider_update_ = ros::WallTime::now();

    if (topic_image_preview_.length() > 0)
        image_dialog_->hide();
}

void RosbagPanel::rangeSliderReleased()
{
    auto& r = play_options_srv_.request;

    float bag_duration = static_cast<float>((end_time_ - start_time_).toSec());

    r.start = bag_duration * getSliderValue(range_slider_, QRangeSlider::HandleStart);
    r.duration = bag_duration * getSliderValue(range_slider_, QRangeSlider::HandleEnd) - r.start;
    r.offset = bag_duration * getSliderValue(range_slider_, QRangeSlider::HandleCurrent) - r.start;
    r.offset = std::max(r.offset, 0.f);
    r.offset = std::min(r.offset, r.duration);

    ros::service::call("/rosbag_play/play_options", play_options_srv_);

    last_slider_update_ = ros::WallTime::now();

    image_dialog_->hide();
}

void RosbagPanel::callbackClock(const rosgraph_msgs::ClockConstPtr& msg)
{
    if (!open_button_->isEnabled())
    {
        // probably another rosbag play is running without "--server" option -> do not update slider
        return;
    }

    // update pause/play button
    if (!prev_time_.isZero())
    {
        if (msg->clock == prev_time_)
        {
            // paused state
            play_pause_button_->setIcon(QIcon::fromTheme("media-playback-start"));
            play_pause_button_->setText("  Play");
            play_options_srv_.request.pause = true;
        }
        else
        {
            // playing state
            play_pause_button_->setIcon(QIcon::fromTheme("media-playback-pause"));
            play_pause_button_->setText("  Pause");
            play_options_srv_.request.pause = false;
        }
    }
    prev_time_ = msg->clock;

    // update sliders
    auto& r = play_options_srv_.request;
    if (!last_slider_update_.isZero() && (ros::WallTime::now() - last_slider_update_).toSec() > 0.5)
    {
        r.offset = static_cast<float>((msg->clock - (start_time_ + ros::Duration(r.start))).toSec());
        setSliderValue(time_slider_, r.offset / r.duration);
        setSliderValue(range_slider_,
                       QRangeSlider::HandleCurrent,
                       (msg->clock - start_time_).toSec() / (end_time_ - start_time_).toSec());
    }
}

void RosbagPanel::callbackBagInfo(const rosbag::BagInfoConstPtr& msg)
{
    ROS_INFO("Got bag info");

    if (!msg->success)
    {
        ROS_ERROR_STREAM(msg->error_message);
        status_label_->setText("Status: Unable to open *.bag files -> " + QString::fromStdString(msg->error_message));
        return;
    }

    status_label_->setText("Status: Successfully opened *.bag files.");

    start_time_ = msg->start;
    end_time_ = msg->end;
    last_bag_info_ = msg;
    auto file_info = QFileInfo(QString::fromStdString(msg->bags[0]));
    bag_folder_ = file_info.dir().absolutePath().toStdString();
    config_path_ = bag_folder_ + "/." + file_info.baseName().toStdString() + ".playerconf";

    // set default play options
    play_options_srv_.request.topics = msg->topic_names;
    play_options_srv_.request.pause_topics = {};
    play_options_srv_.request.advertised_pause_topics = {};
    play_options_srv_.request.start = 0;
    play_options_srv_.request.offset = 0;
    play_options_srv_.request.duration = static_cast<float>((end_time_ - start_time_).toSec());
    play_options_srv_.request.rate = 1.f;
    play_options_srv_.request.pause = true;
    play_options_srv_.request.loop = true;
    play_options_srv_.request.clock = true;
    play_options_srv_.request.hz = 100;
    play_options_srv_.request.quiet = false;
    play_options_srv_.request.prefix = "";
    play_options_srv_.request.immediate = false;
    play_options_srv_.request.queue = 100;
    play_options_srv_.request.delay = 0.2;
    play_options_srv_.request.skip_empty = 3600.f;
    play_options_srv_.request.keep_alive = false;
    play_options_srv_.request.wait_for_subscribers = false;
    play_options_srv_.request.rate_control_topic = "";
    play_options_srv_.request.rate_control_max_delay = 0.f;
    play_options_srv_.request.pause_after_topic = false;

    // overwrite stored play options
    loadConfig();
    checkIfUsePauseTopics(false);

    resetGui();

    // update GUI to loaded play options
    float bag_duration = static_cast<float>((end_time_ - start_time_).toSec());
    float start_ratio = play_options_srv_.request.start / bag_duration;
    float end_ratio = (play_options_srv_.request.start + play_options_srv_.request.duration) / bag_duration;
    range_slider_->setStartAndEnd(static_cast<int>(std::round(start_ratio * RES)),
                                  static_cast<int>(std::round(end_ratio * RES)));
    speed_spin_box_->setValue(play_options_srv_.request.rate);
    if (hide_lower_slider_)
        time_slider_->hide();
    else
        time_slider_->show();

    QList<QStringList> topic_infos = generateTopicInfos("in_bag_image_only");
    if (!topic_infos.empty())
    {
        QString topic_type;
        if (!topic_image_preview_.empty())
        {
            // check if it exists in bag
            auto topic_info = findTopicInfo(topic_image_preview_, topic_infos);
            if (topic_info.empty())
                topic_image_preview_.clear();
            else
                topic_type = topic_info[1];
        }

        if (topic_image_preview_.empty())
        {
            // find new topic (ideally with substring 'front' in it)
            auto it = std::find_if(topic_infos.begin(),
                                   topic_infos.end(),
                                   [](const QStringList& l) { return l[0].contains("front", Qt::CaseInsensitive); });
            if (it != topic_infos.end())
            {
                topic_image_preview_ = (*it)[0].toStdString();
                topic_type = (*it)[1];
            }
            else
            {
                topic_image_preview_ = topic_infos[0][0].toStdString();
                topic_type = topic_infos[0][1];
            }
        }

        if (!topic_image_preview_.empty())
        {
            // send it to dialog
            rosbag::QueryOptions srv;
            srv.request.topics = {topic_image_preview_};
            ros::service::call("/rosbag_play/query_options", srv);
            bool is_compressed = topic_type == "sensor_msgs/CompressedImage";
            image_dialog_->setTopic("/rosbag_play" + topic_image_preview_, is_compressed);
        }
    }
    else
    {
        topic_image_preview_.clear();
    }

    ros::service::call("/rosbag_play/play_options", play_options_srv_);
    if (overwrite_tf_static_from_xml_)
        startTfFromXmlChildProcess();

    disableInputs(false);

    last_slider_update_ = ros::WallTime::now();
}

void RosbagPanel::disableInputs(bool disable)
{
    open_button_->setDisabled(disable);
    speed_spin_box_->setDisabled(disable);
    play_pause_button_->setDisabled(disable);
    options_button_->setDisabled(disable);
    range_slider_->setDisabled(disable);
    time_slider_->setDisabled(disable);
}

void RosbagPanel::openTopicDialog(const std::string& topics_type, std::vector<std::string>& checked_topics)
{
    QList<QStringList> topic_infos = generateTopicInfos(topics_type);

    bool use_flat_list = topics_type == "in_bag_image_only";

    QDialog dialog(this);
    dialog.setWindowTitle("Select desired Topics");

    QGridLayout layout;

    TreeModel tree_model(topic_infos, nullptr, use_flat_list);
    QStringList checked_topics_q;
    for (const auto& t : checked_topics)
        checked_topics_q.push_back(QString::fromStdString(t));
    tree_model.setCheckedTopics(checked_topics_q);
    if (use_flat_list)
        tree_model.setSingleCheckMode(true);
    QTreeView tree_view;
    tree_view.setModel(&tree_model);
    tree_view.expandAll();
    tree_view.resizeColumnToContents(0);
    tree_view.resizeColumnToContents(1);
    tree_view.resizeColumnToContents(2);
    tree_view.collapseAll();
    layout.addWidget(&tree_view);

    QDialogButtonBox button_box(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

    auto* select_button = button_box.addButton("Select All", QDialogButtonBox::ActionRole);
    connect(&button_box, SIGNAL(accepted()), &dialog, SLOT(accept()));
    connect(&button_box, SIGNAL(rejected()), &dialog, SLOT(reject()));
    connect(&button_box,
            &QDialogButtonBox::clicked,
            &dialog,
            [select_button, &tree_model](QAbstractButton* button)
            {
                if (button == select_button)
                {
                    if (button->text() == "Select All")
                    {
                        tree_model.setCheckState(Qt::Checked);
                        button->setText("Deselect All");
                    }
                    else
                    {
                        tree_model.setCheckState(Qt::Unchecked);
                        button->setText("Select All");
                    }
                }
            });
    if (use_flat_list)
        select_button->hide();
    layout.addWidget(&button_box);

    dialog.setLayout(&layout);
    dialog.resize(tree_view.columnWidth(0) + tree_view.columnWidth(1) + tree_view.columnWidth(2) + 30, 400);

    if (dialog.exec())
    {
        checked_topics_q = tree_model.checkedTopics();
        checked_topics.clear();
        for (const auto& t : checked_topics_q)
            checked_topics.push_back(t.toStdString());
    }
}

void RosbagPanel::checkServiceAvailability()
{
    if (ros::service::exists("/rosbag_play/play_options", false))
    {
        if (!open_button_->isEnabled())
        {
            open_button_->setDisabled(false);
            status_label_->setText("Status: Server available.");
        }
    }
    else
    {
        resetGui();
        status_label_->setText("Status: Server not available. Start with: 'rosbag play --server __name:=rosbag_play'");
    }
}

void RosbagPanel::resetGui()
{
    disableInputs(true);

    play_pause_button_->setIcon(QIcon::fromTheme("media-playback-start"));
    play_pause_button_->setText("  Play");

    time_slider_->setRange(0, RES);
    time_slider_->setValue(0);

    range_slider_->setRange(0, RES);
    range_slider_->setStartAndEnd(0, RES);
    range_slider_->setValue(0);

    speed_spin_box_->setRange(0.01, 5.0);
    speed_spin_box_->setSingleStep(0.1);
    speed_spin_box_->setValue(1.);
}

void RosbagPanel::openOptionsDialog()
{
    std::vector<std::string> topics, topics_image_preview;

    topics = play_options_srv_.request.topics;
    if (topic_image_preview_.length() > 0)
        topics_image_preview = {topic_image_preview_};

    QDialog dialog;
    Ui::OptionsDialog ui;
    ui.setupUi(&dialog);
    ui.loop_check_box->setChecked(play_options_srv_.request.loop);
    ui.clock_freq_spin_box->setValue(play_options_srv_.request.hz);
    ui.hide_console_check_box->setChecked(play_options_srv_.request.quiet);
    ui.topix_prefix_line_edit->setText(QString::fromStdString(play_options_srv_.request.prefix));
    ui.immediate_check_box->setChecked(play_options_srv_.request.immediate);
    ui.queue_length_spin_box->setValue(play_options_srv_.request.queue);
    ui.delay_advertise_spin_box->setValue(play_options_srv_.request.delay);
    ui.skip_empty_spin_box->setValue(play_options_srv_.request.skip_empty);
    ui.keep_alive_check_box->setChecked(play_options_srv_.request.keep_alive);
    ui.wait_sub_check_box->setChecked(play_options_srv_.request.wait_for_subscribers);
    ui.rate_control_line_edit->setText(QString::fromStdString(play_options_srv_.request.rate_control_topic));
    ui.rate_control_spin_box->setValue(play_options_srv_.request.rate_control_max_delay);
    ui.pause_after_topic_check_box->setChecked(play_options_srv_.request.pause_after_topic);
    ui.tf_static_xml_check_box->setChecked(overwrite_tf_static_from_xml_);
    ui.hide_lower_slider_check_box->setChecked(hide_lower_slider_);
    connect(ui.topic_push_button, &QPushButton::clicked, [this, &topics]() { openTopicDialog("in_bag", topics); });
    connect(ui.topic_pause_push_button,
            &QPushButton::clicked,
            [this]() { openTopicDialog("in_bag", included_pause_topics_); });
    connect(ui.topic_pause_advertised_push_button,
            &QPushButton::clicked,
            [this]() { openTopicDialog("advertised", advertised_pause_topics_); });
    connect(ui.topic_preview_push_button,
            &QPushButton::clicked,
            [this, &topics_image_preview]() { openTopicDialog("in_bag_image_only", topics_image_preview); });
    if (dialog.exec())
    {
        // do we want to publish tf static from XML
        if (ui.tf_static_xml_check_box->isChecked())
        {
            startTfFromXmlChildProcess();
            topics.erase(std::remove(topics.begin(), topics.end(), "/tf_static"), topics.end());
        }
        else
        {
            stopTfFromXmlChildProcess();
            if (overwrite_tf_static_from_xml_)
            {
                // it was checked before -> automatically add '/tf_static' back
                const auto& all_topics = last_bag_info_->topic_names;
                if (std::find(all_topics.begin(), all_topics.end(), "/tf_static") != all_topics.end())
                    topics.emplace_back("/tf_static");
            }
        }

        // plugin options
        topic_image_preview_ = topics_image_preview.empty() ? "" : topics_image_preview[0];
        overwrite_tf_static_from_xml_ = ui.tf_static_xml_check_box->isChecked();
        hide_lower_slider_ = ui.hide_lower_slider_check_box->isChecked();

        // rosbag play options
        play_options_srv_.request.topics = topics;
        play_options_srv_.request.pause_topics = included_pause_topics_;
        play_options_srv_.request.advertised_pause_topics = advertised_pause_topics_;
        play_options_srv_.request.loop = ui.loop_check_box->isChecked();
        play_options_srv_.request.hz = static_cast<float>(ui.clock_freq_spin_box->value());
        play_options_srv_.request.quiet = ui.hide_console_check_box->isChecked();
        play_options_srv_.request.prefix = ui.topix_prefix_line_edit->text().toStdString();
        play_options_srv_.request.immediate = ui.immediate_check_box->isChecked();
        play_options_srv_.request.queue = ui.queue_length_spin_box->value();
        play_options_srv_.request.delay = static_cast<float>(ui.delay_advertise_spin_box->value());
        play_options_srv_.request.skip_empty = static_cast<float>(ui.skip_empty_spin_box->value());
        play_options_srv_.request.keep_alive = ui.keep_alive_check_box->isChecked();
        play_options_srv_.request.wait_for_subscribers = ui.wait_sub_check_box->isChecked();
        play_options_srv_.request.rate_control_topic = ui.rate_control_line_edit->text().toStdString();
        play_options_srv_.request.rate_control_max_delay = static_cast<float>(ui.rate_control_spin_box->value());
        play_options_srv_.request.pause_after_topic = ui.pause_after_topic_check_box->isChecked();
        checkIfUsePauseTopics(false);
        ros::service::call("/rosbag_play/play_options", play_options_srv_);

        // save new config
        saveConfig();

        // query desired image for preview
        auto topic_info = findTopicInfo(topic_image_preview_, generateTopicInfos("in_bag_image_only"));
        if (!topic_info.empty())
        {
            bool is_compressed = topic_info[1] == "sensor_msgs/CompressedImage";
            rosbag::QueryOptions srv;
            srv.request.topics = topics_image_preview;
            ros::service::call("/rosbag_play/query_options", srv);
            image_dialog_->setTopic("/rosbag_play" + topic_image_preview_, is_compressed);
        }

        // hide lower slider if desired
        if (hide_lower_slider_)
            time_slider_->hide();
        else
            time_slider_->show();
    }
}

QList<QStringList> RosbagPanel::generateTopicInfos(const std::string& topics_type)
{
    QList<QStringList> topic_infos;
    if (topics_type == "in_bag" || topics_type == "in_bag_image_only")
    {
        for (int i = 0; i < last_bag_info_->topic_names.size(); ++i)
        {
            if (topics_type == "in_bag" || last_bag_info_->topic_types[i] == "sensor_msgs/Image" ||
                last_bag_info_->topic_types[i] == "sensor_msgs/CompressedImage")
            {
                QStringList l;
                l.append(QString::fromStdString(last_bag_info_->topic_names[i]));
                l.append(QString::fromStdString(last_bag_info_->topic_types[i]));
                l.append(QString::fromStdString(last_bag_info_->topic_latched[i] ? "true" : "false"));
                topic_infos.append(l);
            }
        }
    }
    else if (topics_type == "advertised")
    {
        ros::master::V_TopicInfo topic_infos_raw;
        if (ros::master::getTopics(topic_infos_raw))
        {
            for (const auto& t : topic_infos_raw)
            {
                QStringList l;
                l.append(QString::fromStdString(t.name));
                l.append(QString::fromStdString(t.datatype));
                l.append("unknown");
                topic_infos.append(l);
            }
        }
    }
    std::sort(
        topic_infos.begin(), topic_infos.end(), [](const QStringList& a, const QStringList& b) { return a[0] < b[0]; });
    return topic_infos;
}

QStringList RosbagPanel::findTopicInfo(const std::string& topic, QList<QStringList> topic_infos)
{
    auto it = std::find_if(
        topic_infos.begin(), topic_infos.end(), [&topic](const QStringList& l) { return l[0].toStdString() == topic; });
    if (it != topic_infos.end())
        return *it;
    else
        return {};
}

void RosbagPanel::saveConfig() const
{
    if (config_path_.empty() || play_options_srv_.request.topics.empty())
        return;

    std::ofstream out(config_path_);
    out << std::fixed << std::setprecision(std::numeric_limits<float>::max_digits10);
    out << "topics: " << join(play_options_srv_.request.topics) << std::endl;
    out << "pause_topics: " << join(included_pause_topics_) << std::endl;
    out << "advertised_pause_topics: " << join(advertised_pause_topics_) << std::endl;
    out << "start: " << play_options_srv_.request.start << std::endl;
    out << "offset: " << play_options_srv_.request.offset << std::endl;
    out << "duration: " << play_options_srv_.request.duration << std::endl;
    out << "rate: " << play_options_srv_.request.rate << std::endl;
    out << "pause: " << static_cast<int>(play_options_srv_.request.pause) << std::endl;
    out << "loop: " << static_cast<int>(play_options_srv_.request.loop) << std::endl;
    out << "clock: " << static_cast<int>(play_options_srv_.request.clock) << std::endl;
    out << "hz: " << play_options_srv_.request.hz << std::endl;
    out << "quiet: " << static_cast<int>(play_options_srv_.request.quiet) << std::endl;
    out << "prefix: " << play_options_srv_.request.prefix << std::endl;
    out << "immediate: " << static_cast<int>(play_options_srv_.request.immediate) << std::endl;
    out << "queue: " << play_options_srv_.request.queue << std::endl;
    out << "delay: " << play_options_srv_.request.delay << std::endl;
    out << "skip_empty: " << play_options_srv_.request.skip_empty << std::endl;
    out << "keep_alive: " << static_cast<int>(play_options_srv_.request.keep_alive) << std::endl;
    out << "wait_for_subscribers: " << static_cast<int>(play_options_srv_.request.wait_for_subscribers) << std::endl;
    out << "rate_control_topic: " << play_options_srv_.request.rate_control_topic << std::endl;
    out << "rate_control_max_delay: " << play_options_srv_.request.rate_control_max_delay << std::endl;
    out << "pause_after_topic: " << static_cast<int>(play_options_srv_.request.pause_after_topic) << std::endl;
    out << "plugin.topic_image_preview: " << topic_image_preview_ << std::endl;
    out << "plugin.overwrite_tf_static_from_xml: " << static_cast<int>(overwrite_tf_static_from_xml_) << std::endl;
    out << "plugin.hide_lower_slider: " << static_cast<int>(hide_lower_slider_) << std::endl;

    out.close();
}

void RosbagPanel::loadConfig()
{
    std::cout << "Config Path: " << config_path_ << std::endl;
    std::ifstream in(config_path_);
    std::string line;
    while (std::getline(in, line))
    {
        auto it = line.find(": ");
        std::string key = line.substr(0, it);
        std::string value = line.substr(it + 2);
        if (key == "topics")
            play_options_srv_.request.topics = split(value);
        else if (key == "pause_topics")
            play_options_srv_.request.pause_topics = included_pause_topics_ = split(value);
        else if (key == "advertised_pause_topics")
            play_options_srv_.request.advertised_pause_topics = advertised_pause_topics_ = split(value);
        else if (key == "start")
            play_options_srv_.request.start = std::stof(value);
        else if (key == "offset")
            play_options_srv_.request.offset = std::stof(value);
        else if (key == "duration")
            play_options_srv_.request.duration = std::stof(value);
        else if (key == "rate")
            play_options_srv_.request.rate = std::stof(value);
        else if (key == "pause")
            play_options_srv_.request.pause = std::stoi(value);
        else if (key == "loop")
            play_options_srv_.request.loop = std::stoi(value);
        else if (key == "clock")
            play_options_srv_.request.clock = std::stoi(value);
        else if (key == "hz")
            play_options_srv_.request.hz = std::stof(value);
        else if (key == "quiet")
            play_options_srv_.request.quiet = std::stoi(value);
        else if (key == "prefix")
            play_options_srv_.request.prefix = value;
        else if (key == "immediate")
            play_options_srv_.request.immediate = std::stoi(value);
        else if (key == "queue")
            play_options_srv_.request.queue = std::stoi(value);
        else if (key == "delay")
            play_options_srv_.request.delay = std::stof(value);
        else if (key == "skip_empty")
            play_options_srv_.request.skip_empty = std::stof(value);
        else if (key == "keep_alive")
            play_options_srv_.request.keep_alive = std::stoi(value);
        else if (key == "wait_for_subscribers")
            play_options_srv_.request.wait_for_subscribers = std::stoi(value);
        else if (key == "rate_control_topic")
            play_options_srv_.request.rate_control_topic = value;
        else if (key == "rate_control_max_delay")
            play_options_srv_.request.rate_control_max_delay = std::stof(value);
        else if (key == "pause_after_topic")
            play_options_srv_.request.pause_after_topic = std::stoi(value);
        else if (key == "plugin.topic_image_preview")
            topic_image_preview_ = value;
        else if (key == "plugin.overwrite_tf_static_from_xml")
            overwrite_tf_static_from_xml_ = std::stoi(value);
        else if (key == "plugin.hide_lower_slider")
            hide_lower_slider_ = std::stoi(value);
        else
            ROS_WARN("Unknown key: %s", key.c_str());
    }
}

std::string RosbagPanel::join(const std::vector<std::string>& v)
{
    std::stringstream ss;
    for (int i = 0; i < v.size(); ++i)
    {
        ss << v[i];
        if (i < v.size() - 1)
            ss << ",";
    }
    return ss.str();
}

std::vector<std::string> RosbagPanel::split(const std::string& s)
{
    std::vector<std::string> v;
    std::stringstream ss(s);
    std::string value;
    while (std::getline(ss, value, ','))
        v.push_back(value);
    return v;
}

void RosbagPanel::startTfFromXmlChildProcess()
{
    stopTfFromXmlChildProcess();

    pid_t pid;
    pid = fork();

    if (pid == -1)
    {
        std::cout << "Unable to fork!" << std::endl;
    }
    else if (pid == 0)
    {
        // automatically stop child process when rviz exits (sometimes rviz crashes and the destructor is not called)
        int r = prctl(PR_SET_PDEATHSIG, SIGINT);
        if (r == -1)
        {
            perror(nullptr);
            return;
        }

        // build desired command
        std::vector<char*> commandVector;
        commandVector.push_back(const_cast<char*>("rosrun"));
        commandVector.push_back(const_cast<char*>("rosbag_panel"));
        commandVector.push_back(const_cast<char*>("publish_tf_static_from_xml.py"));
        commandVector.push_back(const_cast<char*>(bag_folder_.c_str()));
        commandVector.push_back(NULL);

        char** command = commandVector.data();
        execvp(command[0], &command[0]);

        std::cout << "Unable to execvp!" << std::endl;
    }
    else
    {
        pid_pub_tf_static_from_xml_ = pid;
    }
}

void RosbagPanel::stopTfFromXmlChildProcess()
{
    if (pid_pub_tf_static_from_xml_ > 0)
    {
        errno = 0;
        if (kill(pid_pub_tf_static_from_xml_, SIGINT) == 0)
        {
            // successfully sent SIGINT
            int status = 0;
            if (waitpid(pid_pub_tf_static_from_xml_, &status, 0) > 0 && WIFEXITED(status))
            {
                pid_pub_tf_static_from_xml_ = 0;
            }
            else
            {
                std::cout << "Unable to wait for previous process." << std::endl;
                return;
            }
        }
        else if (errno == ESRCH)
        {
            // process not existing anymore
            pid_pub_tf_static_from_xml_ = 0;
        }
        else
        {
            std::cout << "Unable to kill previous process." << std::endl;
            return;
        }
    }
}

void RosbagPanel::save(rviz::Config config) const
{
    Panel::save(config);
    config.mapSetValue("LastOpenedDirectory", QString::fromStdString(join(open_bags_srv_.request.bag_files)));

    // also save custom config next to bag file
    saveConfig();
}

void RosbagPanel::load(const rviz::Config& config)
{
    Panel::load(config);
    QString joined_bag_files;
    config.mapGetString("LastOpenedDirectory", &joined_bag_files);
    open_bags_srv_.request.bag_files = split(joined_bag_files.toStdString());
}

void RosbagPanel::toggleExpandWidget()
{
    if (expand_button_->isChecked())
    {
        expand_button_->setArrowType(Qt::ArrowType::DownArrow);
        expanded_widget_->show();
    }
    else
    {
        expand_button_->setArrowType(Qt::ArrowType::RightArrow);
        expanded_widget_->hide();
    }
}

void RosbagPanel::checkIfUsePauseTopics(bool send)
{
    if (use_pause_topic_check_box_->isChecked())
    {
        play_options_srv_.request.pause_topics = included_pause_topics_;
        play_options_srv_.request.advertised_pause_topics = advertised_pause_topics_;
    }
    else
    {
        play_options_srv_.request.pause_topics.clear();
        play_options_srv_.request.advertised_pause_topics.clear();
    }
    if (send)
        ros::service::call("/rosbag_play/play_options", play_options_srv_);
}

void RosbagPanel::generateFilteredBag()
{
    auto& r = play_options_srv_.request;

    float bag_duration = static_cast<float>((end_time_ - start_time_).toSec());
    std::cout << "BAG FOLDER: " << bag_folder_ << std::endl;
    std::cout << "BAG START: " << r.start << std::endl;
    std::cout << "BAG START: " << r.duration << std::endl;
    std::cout << "TOPICS: " << std::endl;
    for (const auto& topic : r.topics)
        std::cout << "TOPIC: " << topic << std::endl;

    std::string output_filename =
        "/tmp/" +
        QFileInfo(QString::fromStdString(last_bag_info_->bags.back())).baseName().toStdString() + "_filtered.bag";
    FilteredBagGeneration::filter(last_bag_info_->bags, r.start, r.duration, r.topics, output_filename);
}

} // end namespace rosbag_panel

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rosbag_panel::RosbagPanel, rviz::Panel)

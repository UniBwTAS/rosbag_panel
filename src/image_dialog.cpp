#include <QGridLayout>

#include "image_dialog.h"

QImageDialog::QImageDialog(QWidget* parent, const ros::NodeHandle& nh) : QDialog(parent), nh_(nh)
{
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);

    auto* layout = new QGridLayout();
    label_ = new QLabel();
    layout->setMargin(0);
    layout->setSpacing(0);
    layout->addWidget(label_);
    setLayout(layout);
}

void QImageDialog::setTopic(const std::string& topic, bool compressed)
{
    sub_.shutdown();

    if (topic.length() > 0)
    {
        if (!compressed)
            sub_ = nh_.subscribe(topic, 1, &QImageDialog::imageCallback<sensor_msgs::ImageConstPtr>, this);
        else
            sub_ = nh_.subscribe(topic, 1, &QImageDialog::imageCallback<sensor_msgs::CompressedImageConstPtr>, this);
    }
}

template<typename T>
void QImageDialog::imageCallback(const T& msg)
{
    try
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        conversion_mat_ = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("Unable to convert image: " << e.what());
    }

    QImage image(conversion_mat_.data,
                 conversion_mat_.cols,
                 conversion_mat_.rows,
                 conversion_mat_.step[0],
                 QImage::Format_RGB888);
    QPixmap p = QPixmap::fromImage(image);

    label_->setPixmap(p.scaled(600, 600, Qt::KeepAspectRatio));
}

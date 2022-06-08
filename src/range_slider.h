#pragma once

#include <QSlider>
#include <QStyle>
#include <QStyleOptionSlider>
#include <QStylePainter>
#include <QtGui>

class QRangeSlider : public QSlider
{
    Q_OBJECT
  public:
    enum HandleType
    {
        HandleStart,
        HandleEnd,
        HandleCurrent
    };

    QRangeSlider(QWidget* parent = nullptr);

    void mousePressEvent(QMouseEvent* ev) override;
    void mouseMoveEvent(QMouseEvent* ev) override;
    void mouseReleaseEvent(QMouseEvent* ev) override;
    void setStartAndEnd(int start, int end);
    int start() const;
    int current() const;
    int end() const;
    void setValue(int value);
    void setValue(HandleType handle, int value);
    int value() const;
    int value(HandleType handle);
    void setActiveHandle(const HandleType handle);
    HandleType activeHandle();

  protected:
    void paintEvent(QPaintEvent* ev) override;

  private:
    HandleType current_handle_{HandleCurrent};
    int start_{}, end_{}, current_{};
    bool hit_slider_;
};

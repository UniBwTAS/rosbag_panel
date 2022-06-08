#include "range_slider.h"
#include <ros/ros.h>

QRangeSlider::QRangeSlider(QWidget* parent) : QSlider(Qt::Horizontal, parent)
{
}

void QRangeSlider::paintEvent(QPaintEvent* ev)
{
    Q_UNUSED(ev);

    QStylePainter painter(this);
    QStyleOptionSlider opt;
    initStyleOption(&opt);

    // draw groove only
    opt.sliderPosition = 0;
    opt.sliderValue = 0;
    opt.subControls = QStyle::SC_SliderGroove;
    painter.drawComplexControl(QStyle::CC_Slider, opt);

    // get rectangles where to draw handles
    opt.sliderPosition = start_;
    const QRect start_rect = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);
    opt.sliderPosition = end_;
    const QRect end_rect = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);

    // draw complete slider but only between start and end slider (handle is always at max -> end handle)
    QRect prev_slider_rect = opt.rect;
    opt.sliderPosition = maximum();
    opt.sliderValue = maximum();
    opt.subControls = QStyle::SC_SliderGroove | QStyle::SC_SliderHandle;
    opt.rect.setLeft(start_rect.left());
    opt.rect.setRight(end_rect.right());
    painter.drawComplexControl(QStyle::CC_Slider, opt);

    // reset slider rect to full width
    opt.rect = prev_slider_rect;

    // draw start handle only
    opt.subControls = QStyle::SC_SliderHandle;
    opt.sliderPosition = start_;
    opt.sliderValue = start_;
    painter.drawComplexControl(QStyle::CC_Slider, opt);

    // change color of start and end handle
    painter.setPen(Qt::NoPen);
    painter.setBrush(QBrush(QColor(56, 161, 227, 255)));
    painter.drawRect(end_rect);
    painter.drawRect(start_rect);

    // draw current handle only
    opt.sliderPosition = current_;
    opt.sliderValue = current_;
    painter.drawComplexControl(QStyle::CC_Slider, opt);
}

void QRangeSlider::mousePressEvent(QMouseEvent* ev)
{
    if (ev->button() != Qt::LeftButton)
        return;

    QStyleOptionSlider opt;
    initStyleOption(&opt);
    QRect slider_rect;
    hit_slider_ = false;

    // check if end slider was hit
    opt.sliderPosition = end_;
    opt.sliderValue = end_;
    slider_rect = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);
    if (!hit_slider_ && slider_rect.contains(ev->pos()))
    {
        setActiveHandle(HandleEnd);
        setValue(end_);
        hit_slider_ = true;
    }

    // check if current slider was hit
    opt.sliderPosition = current_;
    opt.sliderValue = current_;
    slider_rect = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);
    if (!hit_slider_ && slider_rect.contains(ev->pos()))
    {
        setActiveHandle(HandleCurrent);
        setValue(current_);
        hit_slider_ = true;
    }

    // check if start slider was hit
    opt.sliderPosition = start_;
    opt.sliderValue = start_;
    slider_rect = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);
    if (!hit_slider_ && slider_rect.contains(ev->pos()))
    {
        setActiveHandle(HandleStart);
        setValue(start_);
        hit_slider_ = true;
    }

    if (hit_slider_)
    {
        ev->accept();
        Q_EMIT sliderPressed();
    }
}

void QRangeSlider::mouseMoveEvent(QMouseEvent* ev)
{
    if (!hit_slider_)
        return;

    QStyleOptionSlider opt;
    initStyleOption(&opt);

    QRect groove_rect = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderGroove, this);
    QRect current_handle_rect = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);

    int slider_min = groove_rect.x();
    int slider_max = groove_rect.right() - current_handle_rect.width() + 1;

    setValue(QStyle::sliderValueFromPosition(
        QSlider::minimum(), QSlider::maximum(), ev->pos().x() - slider_min, slider_max - slider_min, opt.upsideDown));

    ev->accept();
    Q_EMIT sliderMoved(current_);
}

void QRangeSlider::mouseReleaseEvent(QMouseEvent* ev)
{
    if (!hit_slider_)
        return;

    ev->accept();
    Q_EMIT sliderReleased();
}

void QRangeSlider::setStartAndEnd(int start, int end)
{
    // reset start and end to avoid clipping in setValue
    start_ = minimum();
    end_ = maximum();

    setActiveHandle(HandleStart);
    setValue(start);

    setActiveHandle(HandleEnd);
    setValue(end);

    setActiveHandle(HandleCurrent);
    setValue(current_);
}

int QRangeSlider::start() const
{
    return start_;
}

int QRangeSlider::current() const
{
    return current_;
}

int QRangeSlider::end() const
{
    return end_;
}

int QRangeSlider::value() const
{
    switch (current_handle_)
    {
        case HandleStart:
            return start_;
        case HandleCurrent:
            return current_;
        case HandleEnd:
            return end_;
        default:
            throw std::invalid_argument("Invalid handle");
    }
}

int QRangeSlider::value(HandleType handle)
{
    HandleType prev_handle = activeHandle();
    setActiveHandle(handle);
    int v = value();
    setActiveHandle(prev_handle);
    return v;
}

void QRangeSlider::setValue(int value)
{
    // ensure valid handle positions
    value = std::max(minimum(), std::min(value, maximum()));

    switch (current_handle_)
    {
        case HandleStart:
            start_ = std::min(end_, value);
            break;
        case HandleCurrent:
            current_ = std::max(start_, std::min(value, end_));
            break;
        case HandleEnd:
            end_ = std::max(start_, value);
            break;
        default:
            throw std::invalid_argument("Invalid handle");
    }
    current_ = std::max(start_, std::min(current_, end_));

    QSlider::setValue(value);
}

void QRangeSlider::setValue(HandleType handle, int value)
{
    HandleType prev_handle = activeHandle();
    setActiveHandle(handle);
    setValue(value);
    setActiveHandle(prev_handle);
}

QRangeSlider::HandleType QRangeSlider::activeHandle()
{
    return current_handle_;
}

void QRangeSlider::setActiveHandle(const HandleType handle)
{
    current_handle_ = handle;
}
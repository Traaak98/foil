#include "slider.h"

Slider::Slider(QWidget *parent) : QSlider(parent) {
    setRange(0, 300);
    connect(this, &QSlider::valueChanged, this, &Slider::onValueChanged);
}

void Slider::onValueChanged(int value) {
    if (value <= 100) {
        emit phaseChanged(value / 100.0);
    } else if (value <= 200) {
        emit frequencyChanged((value - 100) / 100.0);
    } else {
        emit amplitudeChanged((value - 200) / 100.0);
    }
}
#ifndef SLIDER_H
#define SLIDER_H

#include <QSlider>

class Slider : public QSlider {
    Q_OBJECT

public:
    Slider(QWidget *parent = 0);

signals:
    void phaseChanged(double);
    void frequencyChanged(double);
    void amplitudeChanged(double);

private slots:
    void onValueChanged(int value);
};

#endif // SLIDER_H
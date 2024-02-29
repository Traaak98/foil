#include "graphwidget.h"

GraphWidget::GraphWidget(QWidget *parent) : QWidget(parent), phase(0), frequency(1), amplitude(1) {
    QVBoxLayout *layout = new QVBoxLayout(this);
    slider = new Slider();
    plot = new QwtPlot();
    curve = new QwtPlotCurve();
    curve->attach(plot);
    layout->addWidget(plot);
    layout->addWidget(slider);

    connect(slider, &Slider::phaseChanged, this, &GraphWidget::updatePhase);
    connect(slider, &Slider::frequencyChanged, this, &GraphWidget::updateFrequency);
    connect(slider, &Slider::amplitudeChanged, this, &GraphWidget::updateAmplitude);

    updatePlot();
}

void GraphWidget::updatePhase(double phase) {
    this->phase = phase;
    updatePlot();
}

void GraphWidget::updateFrequency(double frequency) {
    this->frequency = frequency;
    updatePlot();
}

void GraphWidget::updateAmplitude(double amplitude) {
    this->amplitude = amplitude;
    updatePlot();
}

void GraphWidget::updatePlot() {
    QVector<QPointF> points;
    for (double x = 0; x < 10; x += 0.1) {
        points.append(QPointF(x, amplitude * sin(frequency * x + phase)));
    }
    curve->setSamples(points);
    plot->replot();
}
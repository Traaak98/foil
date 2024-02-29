#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <QWidget>
#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <QVector>
#include <QPointF>
#include <cmath>
#include "slider.h"

class GraphWidget : public QWidget {
    Q_OBJECT

public:
    GraphWidget(QWidget *parent = 0);

private slots:
    void updatePhase(double phase);
    void updateFrequency(double frequency);
    void updateAmplitude(double amplitude);
    void updatePlot();

private:
    Slider *slider;
    QwtPlot *plot;
    QwtPlotCurve *curve;
    double phase;
    double frequency;
    double amplitude;
};

#endif // GRAPHWIDGET_H
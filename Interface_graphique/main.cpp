// Created by: Moreira, Gustavo
// Created on: 07/07/2021
// Last modified on: 07/07/2021

#include <QApplication>
#include "graphwidget.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    GraphWidget widget;
    widget.show();
    return app.exec();
}

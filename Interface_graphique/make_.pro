QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
TARGET = YourApp
TEMPLATE = app
SOURCES += main.cpp
SOURCES += moc_slider.cpp moc_graphwidget.cpp

INCLUDEPATH+= /usr/include/qwt

HEADERS += slider.h graphwidget.h
SOURCES += main.cpp slider.cpp graphwidget.cpp

HEADERS += slider.h graphwidget.h
SOURCES += main.cpp slider.cpp graphwidget.cpp
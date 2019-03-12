#-------------------------------------------------
#
# Project created by QtCreator 2019-02-18T21:27:47
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG +=c++11

TARGET = ComputerVisionLabs
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    Image.cpp \
    Pyramid.cpp \
    Operation.cpp

HEADERS  += mainwindow.h \
    Image.h \
    Pyramid.h \
    Matrix.h \
    Kernels.h \
    Operation.h

FORMS    += mainwindow.ui

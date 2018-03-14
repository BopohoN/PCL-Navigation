QT += core
QT -= gui

CONFIG += c++11

TARGET = PCLnavi
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    navigation.cpp \
    astar.cpp

LIBS += \
# pcl
        -L/usr/local/lib \
        -L/lib \
        -L/lib/x86_64-linux-gnu \
        -lpcl_io \
        -lpcl_common \
        -lpcl_filters \
        -lpcl_features \
        -lpcl_search \
        -lpcl_segmentation \
        -lpcl_surface \
        -lpcl_visualization \
        -lvtkCommonExecutionModel-6.3 \
        -lvtkCommonCore-6.3 \
        -lvtkCommonDataModel-6.3\
        -lvtkCommonMath-6.3\
        -lvtkRenderingLOD-6.3 \
        -lvtkRenderingCore-6.3 \
        -lvtkFiltersSources-6.3 \
# boost
        -L/usr/lib/x86_64-linux-gnu \
        -lboost_system \
        -lboost_filesystem \


INCLUDEPATH += \
              /usr/include/pcl-1.8/ \
              /usr/include/eigen3/ \
              /usr/include/vtk-6.3/ \

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    navigation.h \
    astar.h

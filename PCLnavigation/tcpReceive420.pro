#-------------------------------------------------
#
# Project created by QtCreator 2018-04-19T18:26:52
#
#-------------------------------------------------

QT       += core gui
QT       += network
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = tcpReceive420
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    showthread.cpp \
    astar.cpp \
    navigation.cpp

HEADERS  += mainwindow.h \
    showthread.h \
    astar.h \
    navigation.h

FORMS    += mainwindow.ui

#PCL
INCLUDEPATH +="/usr/include/pcl-1.8/"
INCLUDEPATH +="/usr/include/eigen3/"
INCLUDEPATH +="/usr/include/boost/"
INCLUDEPATH +="/usr/include/vtk-6.3"

unix|win32: LIBS += -lpcl_common
unix|win32: LIBS += -lpcl_features
unix|win32: LIBS += -lpcl_filters
unix|win32: LIBS += -lpcl_io
unix|win32: LIBS += -lpcl_io_ply
unix|win32: LIBS += -lpcl_kdtree
unix|win32: LIBS += -lpcl_keypoints
unix|win32: LIBS += -lpcl_octree
unix|win32: LIBS += -lpcl_outofcore
unix|win32: LIBS += -lpcl_people
unix|win32: LIBS += -lpcl_recognition
unix|win32: LIBS += -lpcl_registration
unix|win32: LIBS += -lpcl_sample_consensus
unix|win32: LIBS += -lpcl_search
unix|win32: LIBS += -lpcl_segmentation
unix|win32: LIBS += -lpcl_surface
unix|win32: LIBS += -lpcl_tracking
unix|win32: LIBS += -lpcl_visualization
unix|win32: LIBS += -lboost_system
unix|win32: LIBS += -lboost_filesystem
unix|win32: LIBS += -lvtkRenderingCore-6.3
unix|win32: LIBS += -lvtkCommonDataModel-6.3
unix|win32: LIBS += -lvtkCommonMath-6.3
unix|win32: LIBS += -lvtkCommonCore-6.3
unix|win32: LIBS += -lboost_thread
unix|win32: LIBS += -lvtkCommonExecutionModel-6.3
unix|win32: LIBS += -lvtkRenderingLOD-6.3
unix|win32: LIBS += -lvtkFiltersSources-6.3

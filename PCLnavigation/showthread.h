#ifndef SHOWTHREAD_H
#define SHOWTHREAD_H

#include <QThread>
#include <QQueue>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <boost/thread/thread.hpp>

class ShowThread : public QThread
{
    Q_OBJECT

public:
    explicit ShowThread(QObject *parent = 0);

protected:
    QQueue<QString> pointQueue;  //普通点队列
    QQueue<QString> positionQueue;  //路径点队列
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  //普通点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_position;  //路径点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_view;  //用于可视化的点云
    void run();
    void filter();
    void addColor();

public slots:
    void receive(const QString &msg);
    void receivePosition(const QString &pos);
    void receiveOrder(const int order);

signals:
    void returnMsg(const bool isSaved);
};

#endif // SHOWTHREAD_H

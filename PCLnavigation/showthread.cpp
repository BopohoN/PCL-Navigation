#include "showthread.h"
#include <QDebug>
#include <QDateTime>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

ShowThread::ShowThread(QObject *parent)
{
    //----初始化变量----
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_position(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud_view(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud=_cloud;
    cloud_position=_cloud_position;
    cloud_view=_cloud_view;
}

void ShowThread::run()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer.reset(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->addPointCloud(cloud_view,"clouds");

    while(!viewer->wasStopped())
    {
        //----从路径点信息队列中提取点信息进行处理后添加到路径点云中----
        while(!positionQueue.empty())
        {
            QList<QString> positionList=positionQueue.dequeue().split(" ");
            if(positionList.size()==4)
            {
                pcl::PointXYZ basic_point;
                basic_point.x=positionList[1].toDouble();
                basic_point.y=positionList[2].toDouble();
                basic_point.z=positionList[3].toDouble();
                cloud_position->points.push_back(basic_point);
            }
        }

        //----从普通点信息队列中提取点信息进行行处理后添加到普通点云中----
        while(!pointQueue.empty())
        {
            QList<QString> list=pointQueue.dequeue().split('\n');
            for(int i=0;i<list.size();i++)
            {
                //----分割每一行数据  再次分割 得到 x y z 数据----
                QList<QString> pointStr=list[i].split(" ");
                if(pointStr.size()==4)
                {
                    pcl::PointXYZ basic_point;
                    basic_point.x = pointStr[1].toFloat();
                    basic_point.y = pointStr[2].toFloat();
                    basic_point.z = pointStr[3].toFloat();
                    cloud->points.push_back(basic_point);
                }
            }
        }

        filter();  //过滤
        addColor();  //添加颜色
        viewer->updatePointCloud(cloud_view,"clouds");
        viewer->spinOnce(100);
    }

}

/* SLOT: void receive(const QString &msg)
 * 与 SIGNAL: MainWindow::sendPoints(const QString &msg) 关联
 * 获取传入的消息并存入普通点队列
 **/
void ShowThread::receive(const QString &msg)
{
    pointQueue.append(msg);
}

/* SLOT: void receivePosition(const QString &pos)
 * 与 SIGNAL: MainWindow::sendPosition(const QString &pos); 关联
 * 获取传入的消息并存入路径点队列
 **/
void ShowThread::receivePosition(const QString &pos)
{
    positionQueue.append(pos);
}

/* 函数: filter()
 * 使用VoxelGrid滤波器对点云进行下采样
 **/
void ShowThread::filter()
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.03f, 0.03f, 0.03f);    //设置滤波时创建的体素体积
    sor.filter(*cloud);
}

/* 函数：addColor()
 * 给路径点添加红色，给其他点添加白色
 * 添加颜色后将两种点云合并到cloud_view进行可视化
 **/
void ShowThread::addColor()
{
    for(int i=0;i<cloud_position->points.size();i++)
    {
        pcl::PointXYZRGB color_point;
        color_point.x=cloud_position->points[i].x;
        color_point.y=cloud_position->points[i].y;
        color_point.z=cloud_position->points[i].z;
        color_point.r=255;
        color_point.g=0;
        color_point.b=0;
        cloud_view->points.push_back(color_point);
        //qDebug()<<cloud_position->points[i].x<<cloud_position->points[i].y<<cloud_position->points[i].x<<cloud_position->points[i].z;
        cloud_position->clear();
    }
    //pcl::copyPointCloud(*cloud,*cloud_view);
}

/* SLOT: void receiveOrder(const int order)
 * 与 SIGNAL: MainWindow::sendOrder(const int order) 关联
 * 接收命令，执行操作
 * 命令1:保存点云模型，并发送信号 returnMsg(const bool isSaved)返回保存信息
 * */
void ShowThread::receiveOrder(const int order)
{
    QString filename_QStr=QDateTime::currentDateTime().toString("yyyyMMddhhmm")+".pcd";
    std::string filename_str=filename_QStr.toStdString();
    int i;
    switch (order) {
    case 1:
        cloud->resize(cloud->size());

        if(pcl::io::savePCDFileASCII(filename_str,*cloud)==0)
        {
            emit returnMsg(true);
        }
        else
        {
            emit returnMsg(false);
        }
        break;
    default:
        break;
    }
}

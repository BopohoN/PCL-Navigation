/* \author Geoffrey Biggs */
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <QtNetwork>
#include <QString>

using namespace std;
using namespace pcl;

int main()
{
    pcl::PointCloud<pcl::PointXYZ> basic_cloud;

    pcl::PointXYZ point1;
    point1.x = 2;
    point1.y = 2;
    point1.z = 2;
    basic_cloud.points.push_back(point1);
    basic_cloud.height=1;
    basic_cloud.width=1;

    QTcpServer server;
    server.listen(QHostAddress::Any,2048);//change port here
    cout<<"wait for link"<<endl;
    server.waitForNewConnection(100000);
    cout<<"New Connection Established!"<<endl;
    QTcpSocket *socket;
    socket = server.nextPendingConnection();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (basic_cloud.makeShared(), "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while(!viewer->wasStopped()&&socket->state()&&QAbstractSocket::ConnectedState)
    {
        socket->waitForReadyRead(100);
        QString str;
        str=socket->readAll();

        if(str!="")
        {
            QList<QString> pointStr=str.split(" ");
            //分割每一行数据  再次分割 得到 x y z 数据
            if(pointStr.size()==4)
            {
                pcl::PointXYZ point;
                point.x = pointStr[1].toFloat();
                point.y = pointStr[2].toFloat();
                point.z = pointStr[3].toFloat();
                basic_cloud.points.push_back(point);

                viewer->updatePointCloud<pcl::PointXYZ> (basic_cloud.makeShared(), "sample cloud");
            }
        }
        viewer->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    io::savePCDFile("1.pcb",basic_cloud.makeShared());
}

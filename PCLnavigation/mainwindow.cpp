#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "astar.h"
#include "navigation.h"
#include <QFileDialog>
#include <QComboBox>
#include <QtNetwork>
#include <sstream>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    showThread=new ShowThread();

    tcpServer=new QTcpServer(this);
    if(!tcpServer->listen(QHostAddress::Any,6666))  //监听所有网络地址，端口6666
    {
        qDebug()<<tcpServer->errorString();
    }
    connect(tcpServer,SIGNAL(newConnection()),this,SLOT(SocketConnect()));  //绑定信号槽，当有连接时作出反应


    tcpServer_pos=new QTcpServer(this);
    if(!tcpServer_pos->listen(QHostAddress::Any,8888))  //监听所有网络地址，端口8888
    {
        qDebug()<<tcpServer_pos->errorString();
    }
    connect(tcpServer_pos,SIGNAL(newConnection()),this,SLOT(SocketConnect_pos()));

    connect(this,SIGNAL(sendPoints(QString)),showThread,SLOT(receive(QString)));
    connect(this,SIGNAL(sendPosition(QString)),showThread,SLOT(receivePosition(QString)));
    connect(this,SIGNAL(sendOrder(int)),showThread,SLOT(receiveOrder(int)));
    connect(showThread,SIGNAL(returnMsg(bool)),this,SLOT(receiveMsg(bool)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

/* SLOT:void SocketConnect()
 * 与 SIGNAL: newConnection() 关联
 * 当有新的连接接入时调用，获取套接字
 **/
void MainWindow::SocketConnect()
{
    tcpSocket=tcpServer->nextPendingConnection();
    connect(tcpSocket,SIGNAL(readyRead()),this,SLOT(SocketReceive()));
    connect(tcpSocket,SIGNAL(disconnected()),tcpSocket,SLOT(deleteLater()));
    ui->Label_conditionMsg->setText("已连接...");
}

/* SLOT:void SocketReceive()
 * 与 SIGNAL:readyRead() 关联
 * 当Socket准备好接收数据时调用，读取接收的数据
 * 启动线程，并传递数据给线程
 **/
void MainWindow::SocketReceive()
{
    QString recvStr=tcpSocket->readAll();
    //showThread->start();
    emit sendPoints(recvStr);  //发送消息，将数据传入线程
}

void MainWindow::SocketConnect_pos()
{
    tcpSocket_pos=tcpServer_pos->nextPendingConnection();
    connect(tcpSocket_pos,SIGNAL(readyRead()),this,SLOT(SocketReceive_pos()));
    connect(tcpSocket_pos,SIGNAL(disconnected()),tcpSocket_pos,SLOT(deleteLater()));
}

void MainWindow::SocketReceive_pos()
{
    QString recvStr=tcpSocket_pos->readAll();
    showThread->start();
    emit sendPosition(recvStr);
}

/* SLOT:void receiveMsg(const bool isSaved)
 * 与 SIGNAL: ShowThread::returnMsg(const bool isSaved) 关联
 * 接收线程的返回信息，并更新UI显示
 **/
void MainWindow::receiveMsg(const bool isSaved)
{
    if(isSaved)
    {
        ui->label_message->setText("保存成功！");
    }
    else
    {
        ui->label_message->setText("保存失败。");
    }
}


void MainWindow::on_pushButton_saveModel_clicked()
{
    emit sendOrder(1);
}

void MainWindow::on_pushButton_clicked()
{
    QString begin_x,begin_y,begin_z,end_x,end_y,end_z;
    begin_x=ui->lineEdit_start->text();
    begin_y=ui->lineEdit_start_2->text();
    begin_z=ui->lineEdit_start_3->text();
    end_x=ui->lineEdit_end->text();
    end_y=ui->lineEdit_end_2->text();
    end_z=ui->lineEdit_end_3->text();

    path=ui->lineEdit_path->text();

    try
    {
        Navigation n(path.toStdString(),40);

        n.compute(begin_x.toInt(),begin_y.toInt(),begin_z.toInt(),end_x.toInt(),end_y.toInt(),end_z.toInt());

        pcl::visualization::PCLVisualizer cloud_viewer("viewer");
        cloud_viewer.setBackgroundColor(0, 0, 0);
        cloud_viewer.addCoordinateSystem();
        cloud_viewer.addPointCloud(n._cloud_filters);
        for (int i = 0; i < n._road.size(); ++i)
        {
            std::stringstream ss;
            ss<<"cube"<<i;
            cloud_viewer.addCube((float)(n._road[i]._x)/100-0.19, (float)(n._road[i]._x)/100+0.19, (float)(n._road[i]._y)/100-0.19,
               (float)(n._road[i]._y)/100+0.19,(float)(n._road[i]._z)/100-0.19, (float)(n._road[i]._z)/100+0.19,0,1,0,ss.str());
        }
        while(!cloud_viewer.wasStopped())
        {
            cloud_viewer.spinOnce(100);
        }
    }
    catch(int)
    {
        ui->label_error->setText("Couldn't read file.");
    }
    catch(...)
    {
        ui->label_error->setText("Wrong start or end points.");
    }
}

void MainWindow::on_pushButton_pathselect_clicked()
{
    QString directory = QDir::toNativeSeparators(QFileDialog::getOpenFileName(NULL,tr("Open PCD"),".","*.pcd"));
    QString fipath;
    QFileInfo fi;
    fi = QFileInfo(directory);
    fipath = fi.filePath();

    if(!directory.isEmpty())
    {
        ui->lineEdit_path->setText(fipath);
    }
    else
    {
        ui->label_error->setText("Unavailable Path.");
    }
}

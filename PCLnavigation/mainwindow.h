#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "showthread.h"
#include <QMainWindow>
#include <iostream>
#include <string>

class QTcpServer;
class QTcpSocket;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QString path;

private:
    Ui::MainWindow *ui;

    QTcpServer *tcpServer;
    QTcpServer *tcpServer_pos;
    QTcpSocket *tcpSocket;
    QTcpSocket *tcpSocket_pos;
    ShowThread *showThread;

private slots:
    void SocketConnect();
    void SocketReceive();
    void SocketConnect_pos();
    void SocketReceive_pos();
    void receiveMsg(const bool isSaved);
    void on_pushButton_saveModel_clicked();

    void on_pushButton_clicked();

    void on_pushButton_pathselect_clicked();

signals:
    void sendPoints(const QString &msg);
    void sendPosition(const QString &pos);
    void sendOrder(const int order);
};

#endif // MAINWINDOW_H

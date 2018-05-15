#ifndef ASTAR_H
#define ASTAR_H
#include <vector>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/filters/conditional_removal.h>
using namespace std;
struct Node
{
    int _distanceToTarget;
    int _distanceFromBegin;

    int _x;
    int _y;

    Node* _preNode;
    Node():_distanceFromBegin(-1),_distanceToTarget(-1),_x(-1),_y(-1),_preNode(NULL){}
    Node(int x,int y):_distanceFromBegin(-1),_distanceToTarget(-1),_x(x),_y(y),_preNode(NULL){}

};
struct NodePosition
{
    int _x;
    int _y;
    NodePosition():_x(-1),_y(-1){}
    NodePosition(int x,int y):_x(x),_y(y){}
    inline bool operator==(NodePosition n)
    {
        if ((_x==n._x)&&(_y==n._y))
        {
            return true;
        }
        return false;
    }
    inline bool operator!=(NodePosition n)
    {
        if ((_x==n._x)&&(_y==n._y))
        {
            return false;
        }
        return true;
    }
};
struct CubePosition
{
    float _x;
    float _y;
    int _mapX;
    int _mapY;
    inline bool operator==(CubePosition n)
    {
        if ((_mapX==n._mapX)&&(_mapY==n._mapY))
        {
            return true;
        }
        return false;
    }
    inline bool operator!=(CubePosition n)
    {
        if ((_mapX==n._mapX)&&(_mapY==n._mapY))
        {
            return false;
        }
        return true;
    }
    CubePosition():_x(-1),_y(-1),_mapX(-1),_mapY(-1){}
    CubePosition(float x,float y):_x(x),_y(y),_mapX(-1),_mapY(-1){}
    CubePosition(float x,float y,int mapX,int mapY):_x(x),_y(y),_mapX(mapX),_mapY(mapY){}

};
class aStar
{
public:
    aStar();
    aStar(int width,int height);
    aStar(int width,int height,float boxSize);
    aStar(pcl::PointCloud<pcl::PointXYZ> cloud,float boxSize);
    aStar(pcl::PointCloud<pcl::PointXYZ> cloud,float boxSize,float minX,float maxX,float minY,float maxY);
    ~aStar();

    // void setMap(int x,int y,const Node& n);
    // Node getMap(int x,int y);
    void setBeginAndEnd(int beginX,int beginY,int endX,int endY);
    void addBusyBox(int x,int y);
    bool isFindPath();
    void computePath();
    void createCubePath();
    int getPathCost();
    void printPath();

    vector<CubePosition> m_busyCubePosition;
    vector<CubePosition> m_availableCubePosition;
    vector<CubePosition> m_cubePath;
protected:
    void setOpen(int x,int y,bool isOnTheSameRowOrColumn,NodePosition preNode);
    void setClose(int x,int y);
    int m_width;//Number of node s on x axis
    int m_height;//Number of node s on y axis
    float m_nodeSize;//size of node(box)

    NodePosition m_bengin;
    NodePosition m_end;


    vector< vector<Node> > m_map;
    vector<NodePosition> m_openNode;
    vector<NodePosition> m_closeNode;
    vector<NodePosition> m_busyNode;
    vector<NodePosition> m_path;//path from begin to end
    NodePosition m_curComputeNode;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pointCloud;


};
#endif

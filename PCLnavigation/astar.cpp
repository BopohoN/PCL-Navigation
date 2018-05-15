#include "astar.h"
#include <math.h>
#include <algorithm>
aStar::aStar()
{
}
aStar::aStar(pcl::PointCloud<pcl::PointXYZ> cloud,float boxSize)
{
    m_pointCloud=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(cloud, *m_pointCloud);
    float maxX=0;
    float maxY=0;
    float minX=0;
    float minY=0;
    //suppose axis_Z is up
    for (int i = 0; i < m_pointCloud->points.size(); ++i)
    {
        pcl::PointXYZ tempPoint=m_pointCloud->points[i];
        if (tempPoint.x>maxX)
        {
            maxX=tempPoint.x;
        }
        if (tempPoint.x<minX)
        {
            minX=tempPoint.x;
        }
        if (tempPoint.y>maxY)
        {
            maxY=tempPoint.y;
        }
        if (tempPoint.y<minY)
        {
            minY=tempPoint.y;
        }
    }
    float deltaX=maxX- minX;
    float deltaY=maxY- minY;
    deltaX*=100;
    deltaY*=100;
    m_width=(int)(deltaX/boxSize	+0.9f);
    m_height=(int)(deltaY/boxSize +0.9f);

    for (int i = 0; i < m_height; ++i)
    {
        vector<Node> tempNode;
        for (int j = 0; j < m_width; ++j)
        {
            tempNode.push_back(Node(j+1, i+1));
            //setClose(j,i);
        }
        m_map.push_back(tempNode);
    }
    m_nodeSize=boxSize;
    m_curComputeNode=NodePosition(-1,-1);
    for (int i = 0; i < m_height; ++i)
    {
        float cutMinY=(i*boxSize)/100+ minY;
        float cutMaxY=((i+1)*boxSize)/100+minY;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conditionY(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr rand_condY(new pcl::ConditionAnd<pcl::PointXYZ>());
        rand_condY->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
            pcl::FieldComparison<pcl::PointXYZ> ("y",pcl::ComparisonOps::GT,cutMinY)));
        rand_condY->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
            pcl::FieldComparison<pcl::PointXYZ> ("y",pcl::ComparisonOps::LT,cutMaxY)));
        pcl::ConditionalRemoval<pcl::PointXYZ> condRem(false);
        condRem.setCondition(rand_condY);
        condRem.setInputCloud(m_pointCloud);
        condRem.setKeepOrganized(true);
        condRem.filter(*cloud_conditionY);
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(*cloud_conditionY, *cloud_conditionY, mapping);
        for (int j = 0; j < m_width; ++j)
        {
            float cutMinX=(j*boxSize)/100+ minX;
            float cutMaxX=((j+1)*boxSize)/100+minX;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conditionXY(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ConditionAnd<pcl::PointXYZ>::Ptr rand_condXY(new pcl::ConditionAnd<pcl::PointXYZ>());
            rand_condXY->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                pcl::FieldComparison<pcl::PointXYZ> ("x",pcl::ComparisonOps::GT,cutMinX)));
            rand_condXY->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                pcl::FieldComparison<pcl::PointXYZ> ("x",pcl::ComparisonOps::LT,cutMaxX)));
            pcl::ConditionalRemoval<pcl::PointXYZ> condRemXY(false);
            condRemXY.setCondition(rand_condXY);
            condRemXY.setInputCloud(cloud_conditionY);
            condRemXY.setKeepOrganized(true);
            condRemXY.filter(*cloud_conditionXY);
            mapping.clear();
            pcl::removeNaNFromPointCloud(*cloud_conditionXY,*cloud_conditionXY,mapping);
            CubePosition tempPosition;
            tempPosition._x=(cutMinX+cutMaxX)/2;
            tempPosition._y=(cutMinY+cutMaxY)/2;
            tempPosition._mapX=j+1;
            tempPosition._mapY=i+1;
            if (cloud_conditionXY->points.size()>0)
            {
                addBusyBox(j+1, i+1);
                m_busyCubePosition.push_back(tempPosition);
            }
            else
            {
                m_availableCubePosition.push_back(tempPosition);
            }

        }
    }
    std::cout<<"aStar init complete.  width= "<<m_width<<"  height= "<<m_height
        <<"  busyNode count: "<<m_busyNode.size()<<std::endl;


}
aStar::aStar(pcl::PointCloud<pcl::PointXYZ> cloud,float boxSize,float minX,float maxX,float minY,float maxY)
{
    m_pointCloud=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(cloud, *m_pointCloud);
    // maxX/=100;
    // maxY/=100;
    // minX/=100;
    // minY/=100;
    //suppose axis_Z is up
    float deltaX=maxX- minX;
    float deltaY=maxY- minY;
    deltaX*=100;
    deltaY*=100;
    m_width=(int)(deltaX/boxSize	+0.9f);
    m_height=(int)(deltaY/boxSize +0.9f);

    for (int i = 0; i < m_height; ++i)
    {
        vector<Node> tempNode;
        for (int j = 0; j < m_width; ++j)
        {
            tempNode.push_back(Node(j+1, i+1));
            //setClose(j,i);
        }
        m_map.push_back(tempNode);
    }
    m_nodeSize=boxSize;
    m_curComputeNode=NodePosition(-1,-1);
    for (int i = 0; i < m_height; ++i)
    {
        float cutMinY=(i*boxSize)/100+ minY;
        float cutMaxY=((i+1)*boxSize)/100+minY;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conditionY(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr rand_condY(new pcl::ConditionAnd<pcl::PointXYZ>());
        rand_condY->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
            pcl::FieldComparison<pcl::PointXYZ> ("y",pcl::ComparisonOps::GT,cutMinY)));
        rand_condY->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
            pcl::FieldComparison<pcl::PointXYZ> ("y",pcl::ComparisonOps::LT,cutMaxY)));
        pcl::ConditionalRemoval<pcl::PointXYZ> condRem(false);
        condRem.setCondition(rand_condY);
        condRem.setInputCloud(m_pointCloud);
        condRem.setKeepOrganized(true);
        condRem.filter(*cloud_conditionY);
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(*cloud_conditionY, *cloud_conditionY, mapping);
        for (int j = 0; j < m_width; ++j)
        {
            float cutMinX=(j*boxSize)/100+ minX;
            float cutMaxX=((j+1)*boxSize)/100+minX;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conditionXY(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ConditionAnd<pcl::PointXYZ>::Ptr rand_condXY(new pcl::ConditionAnd<pcl::PointXYZ>());
            rand_condXY->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                pcl::FieldComparison<pcl::PointXYZ> ("x",pcl::ComparisonOps::GT,cutMinX)));
            rand_condXY->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                pcl::FieldComparison<pcl::PointXYZ> ("x",pcl::ComparisonOps::LT,cutMaxX)));
            pcl::ConditionalRemoval<pcl::PointXYZ> condRemXY(false);
            condRemXY.setCondition(rand_condXY);
            condRemXY.setInputCloud(cloud_conditionY);
            condRemXY.setKeepOrganized(true);
            condRemXY.filter(*cloud_conditionXY);
            mapping.clear();
            pcl::removeNaNFromPointCloud(*cloud_conditionXY,*cloud_conditionXY,mapping);
            CubePosition tempPosition;
            tempPosition._x=(cutMinX+cutMaxX)/2;
            tempPosition._y=(cutMinY+cutMaxY)/2;
            tempPosition._mapX=j+1;
            tempPosition._mapY=i+1;
            if (cloud_conditionXY->points.size()>0)
            {
                addBusyBox(j+1, i+1);
                m_busyCubePosition.push_back(tempPosition);
            }
            else
            {
                m_availableCubePosition.push_back(tempPosition);
            }

        }
    }
    std::cout<<"aStar init complete.  width= "<<m_width<<"  height= "<<m_height
        <<"  busyNode count: "<<m_busyNode.size()<<std::endl;


}
aStar::aStar(int width,int height)
{
    m_width=width;
    m_height=height;
    for (int i = 0; i < height; ++i)
    {
        vector<Node> tempNode;
        for (int j = 0; j < width; ++j)
        {
            tempNode.push_back(Node(j+1, i+1));
            //setClose(j,i);
        }
        m_map.push_back(tempNode);

    }
    m_nodeSize=10;
    m_curComputeNode=NodePosition(-1,-1);
}
aStar::aStar(int width,int height,float boxSize)
{
    m_width=width;
    m_height=height;
    for (int i = 0; i < height; ++i)
    {
        vector<Node> tempNode;
        for (int j = 0; j < width; ++j)
        {
            tempNode.push_back(Node(j+1, i+1));
            //setClose(j,i);
        }
        m_map.push_back(tempNode);
    }
    m_nodeSize=boxSize;
    m_curComputeNode=NodePosition(-1,-1);
}
aStar::~aStar()
{

}
void aStar::setOpen(int x, int y, bool isOnTheSameRowOrColumn, NodePosition preNode)
{
    if ((x> m_width||x<= 0)||(y> m_height||y<= 0))
    {
        //cout<<"open position error"<<endl;
        return;
    }
    NodePosition targetNode(x,y);
    vector<NodePosition>::iterator it1=std::find(m_busyNode.begin(),
        m_busyNode.end(), targetNode);
    if (it1!=m_busyNode.end())
    {
        return;
    }

    vector<NodePosition>::iterator it2=std::find(m_closeNode.begin(), m_closeNode.end(), targetNode);
    if (it2!=m_closeNode.end())
    {
        return;
    }
    vector<NodePosition>::iterator it3=find(m_openNode.begin(), m_openNode.end(), targetNode);
    if (it3==m_openNode .end())
    {
        m_openNode.push_back(targetNode);

        if (isOnTheSameRowOrColumn)
        {
            m_map[y-1][x-1]._distanceFromBegin=m_map[preNode._y-1][preNode._x-1]._distanceFromBegin+10;
        }
        else
        {
            m_map[y-1][x-1]._distanceFromBegin=m_map[preNode._y-1][preNode._x-1]._distanceFromBegin+14;
        }
        m_map[y-1][x-1]._preNode=&(m_map[preNode._y-1][preNode._x-1]);
        if (targetNode==preNode)
        {
            m_map[y-1][x-1]._distanceFromBegin=0;
            m_map[y-1][x-1]._preNode=NULL;
        }
    }
    else
    {
        int nextDistanceFromBegin;
        int curDistanceFromBegin=m_map[(*it3)._y-1][(*it3)._x-1]._distanceFromBegin;
        if (isOnTheSameRowOrColumn)
        {
            nextDistanceFromBegin=m_map[preNode._y-1][preNode._x-1]._distanceFromBegin+10;
        }
        else
        {
            nextDistanceFromBegin=m_map[preNode._y-1][preNode._x-1]._distanceFromBegin+14;
        }
        if (nextDistanceFromBegin< curDistanceFromBegin)
        {
            m_map[(*it3)._y-1][(*it3)._x-1]._preNode=&(m_map[preNode._y-1][preNode._x-1]);
        }
    }


}
void aStar::setClose(int x, int y)
{
    if ((x> m_width||x<= 0)||(y> m_height||y<= 0))
    {
        cout<<"close position error"<<endl;
        return;
    }
    NodePosition targetNode(x,y);
    vector<NodePosition>::iterator it1=std::find(m_busyNode.begin(),
        m_busyNode.end(), targetNode);
    if (it1!=m_busyNode.end())
    {
        return;
    }
    vector<NodePosition>::iterator  it2=find(m_closeNode.begin(), m_closeNode.end(), targetNode);
    if (it2==m_closeNode.end())
    {
        m_closeNode.push_back(targetNode);
    }
    vector<NodePosition>::iterator it3=find(m_openNode.begin(), m_openNode.end(), targetNode);
    if (it3!=m_openNode.end())
    {
        m_openNode.erase(it3);
    }
}

void aStar::setBeginAndEnd(int beginX, int beginY, int endX, int endY)
{
    if ((beginX>= m_width||beginX< 0)||(beginY>= m_height||beginY< 0))
    {
        cout<<"begin position error"<<endl;
        return;
    }
    if ((endX>= m_width||endX< 0)||(endY>= m_height||endY< 0))
    {
        cout<<"end position error"<<endl;
        return;
    }
    std::vector<CubePosition>::iterator cubeIt=std::find(m_busyCubePosition.begin(), m_busyCubePosition.end(), CubePosition(0, 0, beginX, beginY));
    if (cubeIt!=m_busyCubePosition.end())
    {
        cout<<"begin position busy"<<endl;
        return;
    }
    cubeIt=std::find(m_busyCubePosition.begin(), m_busyCubePosition.end(), CubePosition(0, 0, endX, endY));
    if (cubeIt!=m_busyCubePosition.end())
    {
        cout<<"end position busy"<<endl;
        return;
    }

    m_bengin._x=beginX;
    m_bengin._y=beginY;
    m_map[beginY-1][beginX-1]._distanceFromBegin=0;
    setOpen(beginX, beginY,true,m_bengin);
    m_curComputeNode=m_bengin;
    m_path.push_back(NodePosition(beginX,beginY));
    m_end._x=endX;
    m_end._y=endY;

    for (int i = 0; i < m_height; ++i)
    {
        for (int j = 0; j < m_width; ++j)
        {
            m_map[i][j]._distanceToTarget=abs(m_end._x-1-j)*10+abs(m_end._y-1-i)*10;
        }
    }
}
bool aStar::isFindPath()
{
    vector<NodePosition>::iterator it=find(m_openNode.begin(), m_openNode.end(), m_end);
    if (it!=m_openNode.end())
    {
        return true;
    }
    return false;
}
void aStar::computePath()
{
    while(!isFindPath())
    {
        if (m_curComputeNode._x==-1)
        {
            cout<<"please set begin position!"<<endl;
        }
        setOpen(m_curComputeNode._x, m_curComputeNode._y+1,true,m_curComputeNode);//up
        setOpen(m_curComputeNode._x+1, m_curComputeNode._y,true,m_curComputeNode);//right
        setOpen(m_curComputeNode._x, m_curComputeNode._y-1,true,m_curComputeNode);//down
        setOpen(m_curComputeNode._x-1, m_curComputeNode._y,true,m_curComputeNode);//left
        setOpen(m_curComputeNode._x+1, m_curComputeNode._y+1,false,m_curComputeNode);//rightup
        setOpen(m_curComputeNode._x+1, m_curComputeNode._y-1,false,m_curComputeNode);//rightdown
        setOpen(m_curComputeNode._x-1, m_curComputeNode._y+1,false,m_curComputeNode);//leftup
        setOpen(m_curComputeNode._x-1, m_curComputeNode._y-1,false,m_curComputeNode);//leftdown
        setClose(m_curComputeNode._x, m_curComputeNode._y);
        if (m_openNode.size()==0)
        {
            cout<<"can't find path!"<<endl;
            break;
        }
        int curMinToTarget=m_map[m_openNode[0]._y-1][m_openNode[0]._x-1]._distanceToTarget
            +m_map[m_openNode[0]._y-1][m_openNode[0]._x-1]._distanceFromBegin;
        NodePosition nextNode(m_openNode[0]._x,m_openNode[0]._y);
        for(vector<NodePosition>::iterator i=m_openNode.begin();i!=m_openNode.end();i++)
        {
            if (abs((*i)._x- m_curComputeNode._x)>1||abs((*i)._y- m_curComputeNode._y)>1)
            {
                continue;
            }
            int curDistance=m_map[(*i)._y-1][(*i)._x-1]._distanceFromBegin+m_map[(*i)._y-1][(*i)._x-1]._distanceToTarget;
            if (curDistance<curMinToTarget)
            {
                curMinToTarget=curDistance;
                nextNode._x=(*i)._x;
                nextNode._y=(*i)._y;
            }
        }
        m_curComputeNode._x=nextNode._x;
        m_curComputeNode._y=nextNode._y;
    }

}
void aStar::addBusyBox(int x, int y)
{
    vector<NodePosition>::iterator it=std::find(m_busyNode.begin(), m_busyNode.end(), NodePosition(x, y));
    if (it==m_busyNode.end())
    {
        m_busyNode.push_back(NodePosition(x, y));
    }
}
void aStar::createCubePath()
{
    if (!isFindPath())
    {
        cout<<"don't find path"<<endl;
    }
    else
    {
        Node *pTempNode=&(m_map[m_end._y-1][m_end._x-1]);
        while(pTempNode)
        {
            CubePosition tempCubePosition;
            tempCubePosition._mapX=pTempNode->_x;
            tempCubePosition._mapY=pTempNode->_y;
            vector<CubePosition>::iterator it=std::find(m_availableCubePosition.begin(),
                m_availableCubePosition.end(), tempCubePosition);
            tempCubePosition._x=(*it)._x;
            tempCubePosition._y=(*it)._y;
            m_availableCubePosition.erase(it);
            m_cubePath.push_back(tempCubePosition);
            pTempNode=(*pTempNode)._preNode;
        }
    }
}
void aStar::printPath()
{
    if (!isFindPath())
    {
        cout<<"don't find path"<<endl;
    }
    else
    {
        Node *pTempNode=&(m_map[m_end._y-1][m_end._x-1]);
        while(pTempNode)
        {
            cout<<"x: "<<(*pTempNode)._x<<"  y: "<<(*pTempNode)._y<<endl;
            pTempNode=(*pTempNode)._preNode;
        }
    }
}
int aStar::getPathCost()
{
    if (!isFindPath())
    {
        return -1;
    }
    else
    {
        return m_map[m_end._y-1][m_end._x-1]._distanceToTarget+m_map[m_end._y-1][m_end._x-1]._distanceFromBegin;
    }
}

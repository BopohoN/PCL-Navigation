#include "navigation.h"
Navigation::Navigation(std::string objPath,int boxSize)
{
    //---------------------If you use OBJ,uncomment this----------------------------//
    //OBJ to PCD
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileOBJ(objPath, mesh);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud_original);
    for (int i = 0; i < cloud_original->points.size() ; ++i)
    {
        cloud_original->points[i].r=255;
        cloud_original->points[i].g=255;
        cloud_original->points[i].b=255;
    }
    _cloud_original=cloud_original;

    _boxSize=boxSize;
    _minCostCountZ=-99;
    //pcl::io::savePCDFileASCII("original.pcd", *_cloud_original);
    //-------------------------------------------------------------------------------//

    //load PCD File from PCL_TCPProcess,if you use PCD,uncomment this.
    /*pcl::io::loadPCDFile(objPath,*_cloud_original);*/

    //Magic,don't touch this.
    _boxSize=boxSize;
    _minCostCountZ=-99;
}
Navigation::~Navigation()
{
}
void Navigation::voxelGrid()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filters(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(_cloud_original);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filters);
    _cloud_filters=cloud_filters;
}
void Navigation::statisticalOutlierRemoval()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filters(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(_cloud_filters);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filters);
    _cloud_filters=cloud_filters;
}
void Navigation::computeNormal()
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(_cloud_filters);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne;
    ne.setInputCloud(_cloud_filters);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.1f);
    ne.compute(*cloud_normal);
    pcl::copyPointCloud(*_cloud_filters, *cloud_normal);
    _cloud_normal=cloud_normal;
}
void Navigation::segmentationByNormal()
{
    //build conditionliu
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_verticalPlaneNormal(new pcl::PointCloud<pcl::PointNormal>);
    pcl::ConditionAnd<pcl::PointNormal>::Ptr rand_cond(new pcl::ConditionAnd<pcl::PointNormal>());
    rand_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
        pcl::FieldComparison<pcl::PointNormal> ("normal_z",pcl::ComparisonOps::LT,0.95)));
    rand_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
        pcl::FieldComparison<pcl::PointNormal>("normal_z",pcl::ComparisonOps::GT,-0.95)));

    pcl::ConditionalRemoval<pcl::PointNormal> condRem(rand_cond);
    condRem.setInputCloud(_cloud_normal);
    condRem.setKeepOrganized(true);
    condRem.filter(*cloud_verticalPlaneNormal);
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud_verticalPlaneNormal, *cloud_verticalPlaneNormal, mapping);
    std::cout<<"cloud_verticalPlaneNormal points count: "<<cloud_verticalPlaneNormal->points.size()<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vertical(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*_cloud_filters, *cloud_vertical);
    pcl::copyPointCloud(*cloud_verticalPlaneNormal, *cloud_vertical);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_horizontalPlaneNormal(new pcl::PointCloud<pcl::PointNormal>);
    pcl::ConditionOr<pcl::PointNormal>::Ptr rand_cond2(new pcl::ConditionOr<pcl::PointNormal>());
    rand_cond2->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
        pcl::FieldComparison<pcl::PointNormal> ("normal_z",pcl::ComparisonOps::GT,0.85)));
    rand_cond2->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
        pcl::FieldComparison<pcl::PointNormal>("normal_z",pcl::ComparisonOps::LT,-0.85)));

    pcl::ConditionalRemoval<pcl::PointNormal> condRem2(rand_cond2);
    condRem2.setInputCloud(_cloud_normal);
    condRem2.setKeepOrganized(true);
    condRem2.filter(*cloud_horizontalPlaneNormal);
    std::vector<int> mapping2;
    pcl::removeNaNFromPointCloud(*cloud_horizontalPlaneNormal, *cloud_horizontalPlaneNormal, mapping2);
    std::cout<<"cloud_horizontalPlaneNormal points count: "<<cloud_horizontalPlaneNormal->points.size()<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_horizontal(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*_cloud_filters, *cloud_horizontal);
    pcl::copyPointCloud(*cloud_horizontalPlaneNormal, *cloud_horizontal);

    _cloud_vertical=cloud_vertical;
    _cloud_horizontal=cloud_horizontal;

    //pcl::io::savePCDFileASCII("vertical.pcd", *_cloud_vertical);
    //pcl::io::savePCDFileASCII("horizontal.pcd", *_cloud_horizontal);
}
void Navigation::cutByVertical()
{
    for (int i = 1; i < _sizeZ+1; ++i)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pt;
        pt.setNegative(false);
        pt.setFilterFieldName("z");
        if (i==_sizeZ)
        {
            pt.setFilterLimits(_minZ+ (float)_boxSize*(i-1)/100, _maxZ);
        }
        else
        {
            pt.setFilterLimits(_minZ+(float)_boxSize*(i-1)/100,_minZ+(float)_boxSize*i/100);
        }
        pt.setInputCloud(_cloud_vertical);
        pt.filter(*tempCloud);
        _clouds_vertical.push_back(tempCloud);

        //project
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        coefficients->values.resize(4);
        coefficients->values[0]=coefficients->values[1]=coefficients->values[3]=0;
        coefficients->values[2]=1;
        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(tempCloud);
        proj.setModelCoefficients(coefficients);
        proj.filter(*tempCloud);
        _clouds_verticalProjs.push_back(tempCloud);
        std::stringstream ss;
        ss<<"v"<<i<<".pcd";
        //pcl::io::savePCDFileASCII(ss.str(), *tempCloud);


    }
}
void Navigation::cutByHorizontal()
{
    //suppose y_axis is up

    for (int i = 1; i < _sizeX+1; ++i)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pt;
        pt.setNegative(false);
        pt.setFilterFieldName("x");
        if (i==_sizeX)
        {
            pt.setFilterLimits(_minX+(float)_boxSize*(i-1)/100, _maxX);
        }
        else
        {
            pt.setFilterLimits(_minX+(float)_boxSize*(i-1)/100,_minX+(float)_boxSize*i/100);
        }
        pt.setInputCloud(_cloud_horizontal);
        pt.filter(*tempCloud);
        _clouds_horizontal.push_back(tempCloud);

        //project
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        coefficients->values.resize(4);
        coefficients->values[2]=coefficients->values[1]=coefficients->values[3]=0;
        coefficients->values[0]=1;
        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(tempCloud);
        proj.setModelCoefficients(coefficients);
        proj.filter(*tempCloud);
        _clouds_horizontalProjs.push_back(tempCloud);
        std::stringstream ss;
        ss<<"h"<<i<<".pcd";
        //pcl::io::savePCDFileASCII(ss.str(), *tempCloud);
    }
}
void Navigation::setBeginEnd(int x0, int y0, int z0, int x1, int y1, int z1)
{
    //suppose z_axis is up, x_axis is forward, y_axis is right
    float minX=0;
    float maxX=0;
    float minY=0;
    float maxY=0;
    float minZ=0;
    float maxZ=0;
    for (int i = 0; i < _cloud_filters->points.size(); ++i)
    {
        float temp=_cloud_filters->points[i].x;
        if (temp>maxX)
        {
            maxX=temp;
        }
        if (temp<minX)
        {
            minX=temp;
        }
        temp=_cloud_filters->points[i].y;
        if (temp>maxY)
        {
            maxY=temp;
        }
        if (temp<minY)
        {
            minY=temp;
        }
        temp=_cloud_filters->points[i].z;
        if (temp>maxZ)
        {
            maxZ=temp;
        }
        if (temp<minZ)
        {
            minZ=temp;
        }
    }
    _minX=minX;
    _minY=minY;
    _minZ=minZ;
    _maxX=maxX;
    _maxY=maxY;
    _maxZ=maxZ;

    _sizeX=(int)((_maxX-_minX)*100)/_boxSize+0.9;
    _sizeY=(int)((_maxY-_minY)*100)/_boxSize+0.9;
    _sizeZ=(int)((_maxZ-_minZ)*100)/_boxSize+0.9;



    _beginPosition._x=x0 ;
    _beginPosition._y=y0 ;
    _beginPosition._z=z0 ;

    _endPosition._x=x1 ;
    _endPosition._y=y1 ;
    _endPosition._z=z1 ;
}
void Navigation::computeHorizontalRoad()
{
    // x is right,y is forward
    int beginX=_beginPosition._x/_boxSize+1;
    int beginY=_beginPosition._y/_boxSize+1;
    int endX=_endPosition._x/_boxSize+1;
    int endY=_endPosition._y/_boxSize+1;
    int curCountZ=_beginPosition._z/_boxSize;
    int endCountZ=_endPosition._z/_boxSize;
    int minCost=99999;
    for (int i = 0; i < _clouds_verticalProjs.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_aStar;
        pcl::copyPointCloud(*(_clouds_verticalProjs[i]),cloud_aStar);
        aStar as(cloud_aStar, _boxSize, _minX, _maxX, _minY, _maxY);
        as.setBeginAndEnd(beginX, beginY, endX, endY);
        as.computePath();
        as.createCubePath();
        as.printPath();
        int curCost=as.getPathCost();
        if (curCost!=-1&&curCost<minCost)
        {
            minCost=curCost;
            _minCostCountZ=i;
            _pathAStar=as;
        }
        else if (curCost!=-1&&curCost==minCost)
        {
            if ((abs(i- curCountZ)+abs(i- endCountZ))
                <(abs(_minCostCountZ- curCountZ)+abs(_minCostCountZ- endCountZ)))
            {
                _minCostCountZ=i;
                _pathAStar=as;
            }
        }
        std::cout<<"minCost: "<<minCost<<" minCountZ: "<<_minCostCountZ<<std::endl;

    }
}
void Navigation::computeVerticalRoad()
{
    //begin
    //x:right y:up
    int beginX=_beginPosition._y/_boxSize+1;
    int beginY=_beginPosition._z/_boxSize+1;
    int endX=_beginPosition._y/_boxSize+1;
    int endY=_minCostCountZ+1;
    int curCountX=_beginPosition._x/_boxSize;
    int endCountX=_endPosition._x/_boxSize;

    std::cout<<"curCountX: "<<curCountX<<std::endl;

    pcl::PointCloud<pcl::PointXYZ> cloud_aStar;
    //pcl::copyPointCloud(*(_clouds_horizontalProjs[curCountX]),cloud_aStar);
    cloud_aStar.width=(*(_clouds_horizontalProjs[curCountX])).width;
    cloud_aStar.height=(*(_clouds_horizontalProjs[curCountX])).height;
    cloud_aStar.points.resize(cloud_aStar.width*cloud_aStar.height);
    for (int i = 0; i < (*(_clouds_horizontalProjs[curCountX])).points.size(); ++i)
    {
        cloud_aStar.points[i].x=(*(_clouds_horizontalProjs[curCountX])).points[i].y;
        cloud_aStar.points[i].y=(*(_clouds_horizontalProjs[curCountX])).points[i].z;
    }
    aStar asBegin(cloud_aStar,_boxSize, _minY, _maxY, _minZ, _maxZ);
    asBegin.setBeginAndEnd(beginX, beginY, endX, endY);
    asBegin.computePath();
    asBegin.createCubePath();
    asBegin.printPath();
    _beginAStar=asBegin;

    //end
    beginX=_endPosition._y/_boxSize+1;
    beginY=_minCostCountZ+1;
    endX=_endPosition._y/_boxSize+1;
    endY=_endPosition._z/_boxSize+1;
    //pcl::copyPointCloud(*(_clouds_horizontalProjs[endCountX]),cloud_aStar);
    cloud_aStar.width=(*(_clouds_horizontalProjs[endCountX])).width;
    cloud_aStar.height=(*(_clouds_horizontalProjs[endCountX])).height;
    cloud_aStar.points.resize(cloud_aStar.width*cloud_aStar.height);
    for (int i = 0; i < (*(_clouds_horizontalProjs[endCountX])).points.size(); ++i)
    {
        cloud_aStar.points[i].x=(*(_clouds_horizontalProjs[endCountX])).points[i].y;
        cloud_aStar.points[i].y=(*(_clouds_horizontalProjs[endCountX])).points[i].z;
    }
    aStar asEnd(cloud_aStar,_boxSize, _minY, _maxY, _minZ, _maxZ);
    asEnd.setBeginAndEnd(beginX, beginY, endX, endY);
    asEnd.computePath();
    asEnd.createCubePath();
    asEnd.printPath();
    _endAStar=asEnd;


}
void Navigation::createRoad()
{
    if (_beginAStar.isFindPath()&&_pathAStar.isFindPath()&&_endAStar.isFindPath())
    {
        int curCountX=_beginPosition._x/_boxSize;
        int endCountX=_endPosition._x/_boxSize;
        //begin road
        for (int i = 0; i < _beginAStar.m_cubePath.size(); ++i)
        {
            Vector3 tempPosition(_minX*100+curCountX* _boxSize+0.5* _boxSize,_beginAStar.m_cubePath[i]._x*100 ,
                _beginAStar.m_cubePath[i]._y*100);
            _road.push_back(tempPosition);
        }
        //path road
        for (int i = 0; i < _pathAStar.m_cubePath.size(); ++i)
        {
            // Vector3 tempPosition((_pathAStar.m_cubePath[i]._mapX-1)*_boxSize+_minX*100 +0.5* _boxSize,(_pathAStar.m_cubePath[i]._mapY-1)*_boxSize+_minY*100+0.5* _boxSize ,
            // 	_minZ*100+_minCostCountZ*_boxSize+0.5* _boxSize);
            Vector3 tempPosition(_pathAStar.m_cubePath[i]._x*100 ,_pathAStar.m_cubePath[i]._y*100 ,
                _minZ*100+_minCostCountZ*_boxSize+0.5* _boxSize);
            _road.push_back(tempPosition);
            // _road.push_back(tempPosition2);
        }
        //end road
        for (int i = 0; i < _endAStar.m_cubePath.size(); ++i)
        {
            Vector3 tempPosition(_minX*100+endCountX* _boxSize+0.5* _boxSize,_endAStar.m_cubePath[i]._x*100 ,
                _endAStar.m_cubePath[i]._y*100);
            _road.push_back(tempPosition);
        }
    }
    else
    {
        return;
    }
}
void Navigation::compute(int x0, int y0, int z0, int x1, int y1, int z1)
{
    voxelGrid();
    statisticalOutlierRemoval();
    setBeginEnd(x0,y0,z0,x1,y1,z1);
    computeNormal();
    segmentationByNormal();
    cutByVertical();
    cutByHorizontal();
    computeHorizontalRoad();
    computeVerticalRoad();
    createRoad();
}

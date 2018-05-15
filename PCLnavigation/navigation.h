#ifndef NAVIGATION_H
#define NAVIGATION_H
#include <iostream>
#include <string>
#include <sstream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include "astar.h"
struct Vector3
{
    float _x;
    float _y;
    float _z;
    Vector3():_x(0),_y(0),_z(0){}
    Vector3(float x,float y,float z):_x(x),_y(y),_z(z){}
};
class Navigation
{
public:
    //Navigation();
    Navigation(std::string objPath,int boxSize);
    ~Navigation();
    //start:x0,y0,z0 end:x1,y1,z1
    void compute(int x0,int y0,int z0,int x1,int y1,int z1);



    std::vector<Vector3> _road;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud_filters;

protected:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud_original;
    pcl::PointCloud<pcl::PointNormal>::Ptr _cloud_normal;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud_vertical;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud_horizontal;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _clouds_vertical;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _clouds_verticalProjs;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _clouds_horizontal;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _clouds_horizontalProjs;
    aStar _beginAStar;
    aStar _pathAStar;
    aStar _endAStar;

    void voxelGrid();
    void statisticalOutlierRemoval();
    void computeNormal();
    void segmentationByNormal();
    void cutByVertical();
    void cutByHorizontal();
    void setBeginEnd(int x0,int y0,int z0,int x1,int y1,int z1);
    void computeVerticalRoad();
    void computeHorizontalRoad();
    void createRoad();



    float _boxSize;
    float _boxHeight;
    int _minCostCountZ;//which count of z has the minimum cost
                        //(this value can use as _clouds_verticalProjs[_minCostCountZ])
    float _minX;//meter
    float _minY;//meter
    float _minZ;//meter
    float _maxX;//meter
    float _maxY;//meter
    float _maxZ;//meter
    int _sizeX;
    int _sizeY;
    int _sizeZ;
    Vector3 _beginPosition;
    Vector3 _endPosition;

};
#endif

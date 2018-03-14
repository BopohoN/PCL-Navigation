#include <iostream>
#include <string>
#include <sstream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include "astar.h"
#include "navigation.h"
int main(int argc, char const *argv[])
{
    Navigation n("/*change obj or pcd path here*/",40);
    n.compute(165, 85, 50, 560, 600, 90);

    pcl::visualization::PCLVisualizer cloud_viewer("viewer");
    cloud_viewer.setBackgroundColor(0.3, 0.3, 0.3);
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
        cloud_viewer.spin();
    }
}

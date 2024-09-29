/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


// void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
// {
//     // ----------------------------------------------------
//     // -----Open 3D viewer and display simple highway -----
//     // ----------------------------------------------------
    
//     // RENDER OPTIONS
//     bool renderScene = false;
//     std::vector<Car> cars = initHighway(renderScene, viewer);
    
//     // TODO:: Create lidar sensor 
//     Lidar *lidar = new Lidar(cars, 0.0f);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pc = lidar->scan();
//     // renderRays(viewer, lidar->position, pc);
//     renderPointCloud(viewer, pc, "cloud");
//     // TODO:: Create point processor
//     ProcessPointClouds<pcl::PointXYZ> ppc;
//     std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>  segmented = ppc.SegmentPlane(pc, 1000, 0.2);

//     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = ppc.Clustering(segmented.first, 1.0, 3, 30);
//     // renderPointCloud(viewer,segmented.first,"obstCloud",Color(1,0,0));
//     // renderPointCloud(viewer,segmented.second,"planeCloud",Color(0,1,0));
	
//     int clusterId = 0;
//     std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    
//     for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
//     {
//         std::cout << "cluster size ";
//         ppc.numPoints(cluster);
//         renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
//         Box box = ppc.BoundingBox(cluster);
//         renderBox(viewer,box,clusterId);

//         ++clusterId;

//     }

// }


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}



void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // Experiment with the ? values and find what works best
    Box box;
    box.x_min = -10;
    box.y_min = -5;
    box.z_min = -2;
    box.x_max = 30;
    box.y_max = 7;
    box.z_max = 1.f;
    Eigen::Vector4f minPoint(box.x_min,box.y_min,box.z_min,1);
    Eigen::Vector4f maxPoint(box.x_max,box.y_max,box.z_max,1);

        Box carbox;
    carbox.x_min = -1.5f;
    carbox.y_min = -1.2f;
    carbox.z_min = -2.f;
    carbox.x_max = 2.5f;
    carbox.y_max = 1.2f;
    carbox.z_max = 0.2f;

    typename pcl::PointCloud<pcl::PointXYZI>::Ptr pc = pointProcessorI->FilterCloud(inputCloud, 0.25f , minPoint, maxPoint);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>  segmented = pointProcessorI->SegmentPlane(pc, 25, .25);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmented.first, 0.4, 8, 400);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    renderPointCloud(viewer,segmented.first,"obstacles", Color(0,1,1));
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        // std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);

        ++clusterId;
    }

    // renderBox(viewer, box, clusterId+1, Color(0,0.5,1), 0.7f);
    renderBox(viewer, carbox, clusterId+2, Color(0,1,1), 0.5f);
    renderPointCloud(viewer,segmented.second,"road", Color(1,1,0));
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    cityBlock(viewer, pointProcessorI, inputCloud);
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // cityBlock(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);
            
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }

}
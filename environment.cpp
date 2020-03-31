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

/*
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars,0);//added ash
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //creating point cloud to store data that the lidar is looking at
    //renderRays(viewer, lidar->position, inputCloud);
    //visualize the created rays-> where lidar is located (origin)as all the rays are starting from there ->storing in created point cloud above
    renderPointCloud(viewer, inputCloud, "inputCloud");//here we are trying to see the pointcloud data 
    //visualize-> stored in point cloud->color
   // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    //Call cluster function
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster :cloudClusters)
    {
        //if(render_clusters)
        {
        std::cout << "cluster size";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstcloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
        }

        //if(render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
    
        clusterId++;
    }
}*/

//TO DO Create a Real PCD data
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    //ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");//load real pcd file to point processor
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor.FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-10, -6, -3, 1), Eigen::Vector4f (30, 6, 4, 1));
    renderPointCloud(viewer, filterCloud, "filterCloud");//render (view) the inputcloud, named inputcloud
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(filterCloud, 40, 0.3);
    renderPointCloud(viewer,segmentCloud.first,"obstacleCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    //Call cluster function
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 0.5, 5, 300);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster :cloudClusters)
    {
        //if(render_clusters)
        {
        std::cout << "cluster size";
        pointProcessor.numPoints(cluster);//call function numpoints to display the points in each cluster
        renderPointCloud(viewer,cluster,"obstcloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
        }

        //if(render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);//call function boundingboxes to create box around each cluster
            renderBox(viewer, box, clusterId);
        }
    
        clusterId++;
    }

}

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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI; //create point processor using XYZI type of point clouds
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");//stream real pcd file to point processor
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;//Create a inputcloud to point towards the pointcloud data*/

    while (!viewer->wasStopped ())
    {
        //Clear Viewer
       viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        //Loadthe PCD and run the obstacle detection
        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}

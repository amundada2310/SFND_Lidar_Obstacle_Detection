// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction// this will provide for 1 single point for each voxelgrid cube defined by the filterRes
    pcl::VoxelGrid<PointT>  vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloudFiltered);

    //TO DO::fill in function for region based filtering //cropbox filtering only the region left where we want lidar to look at
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    //TO DO:: fill in function to reomve roofs
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);    

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
    inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    //TO DO: Create 2 different clouds 1 Obstacles, 2 Ground
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT> ());//create obstacle point cloud on heap
    typename pcl::PointCloud<PointT>::Ptr groundCloud (new pcl::PointCloud<PointT> ());//create ground(road) point cloud on heap

    for(int index : inliers->indices)
    {
        groundCloud->points.push_back(cloud->points[index]);//all inliers are plane/ground point cloud
    }

    pcl::ExtractIndices<PointT> extract;//extracts the obstacle point cloud from the ground point cloud
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacleCloud);//everything besides the inliers are kept and are assigned to obstacle point cloud

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, groundCloud);
    return segResult;//returns everything to SegmentPlane function

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients()};
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices()};
    // TODO:: Fill in this function to find inliers for the cloud.

    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    //seg.setMethodType(pcl::SAC_RANSAC); 
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    //seg.segment (*inliers, *coefficients);

    
    std::unordered_set<int> inliersResult; //inliersResult initialized as 0
    srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	while(maxIterations--)
    {
        //randomly select 3 points from the cloud
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
		inliers.insert(rand()%(cloud->points.size()));
		//rand function will randomly select values from (%) '0' to max point.size(max no. of points) and values are stored (inserted) in inliers

		//x1,y1,z1,x2,y2,z2,x3,y3,z3 holding values of the 3 randomly selected points

		float x1,y1,z1,x2,y2,z2,x3,y3,z3;
		auto itr = inliers.begin(); //itr is a pointer pointing to the first location of inliers (first point)
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

		float a = (((y2-y1)*(z3-z1))-((z2-z1)*(y3-y1)));
		float b = (((z2-z1)*(x3-x1))-((x2-x1)*(z3-z1)));
		float c = (((x2-x1)*(y3-y1))-((y2-y1)*(x3-x1)));
        float d = -(a*x1+b*y1+c*z1);
            
        //Iterate through all the points to calcualte the tolerance and find the np. of inliers and select the best model
        for(int index = 0; index < cloud->points.size(); index++)
        {
            //If the index is pointing to a point which is already part of plane we have to continue (not consider the point)
			//There are 3 such points.

			if(inliers.count(index)>0)
			continue;

			//if it is not the 3 points then do as below
			//pointing the point values in x4 and y4, z4
			pcl::PointXYZI point;
			point = cloud->points[index];
			
			float x4 = point.x;
			float y4 = point.y;
            float z4 = point.z;
			//now calculating distance from plane to that point
			float e = fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);

			if(e<distanceThreshold)
			inliers.insert(index);
        }

		//when the tolerance is calculated for all the points for 1 particular point combination selected
		//then its size is compared with size of inlierResult (which is initialized as 0). If the inliers size is >
		//then store that value in result

		if(inliers.size()>inliersResult.size())
        {
            inliersResult = inliers;
        }

    } 

    for(int point : inliersResult)
    inliers->indices.push_back(point);

    //OR//

    /*for(int i = 0; i <= cloud->points.size(); i++)
    {
        if(inliersResult.count(i))
        inliers->indices.push_back(cloud->points(i));//push back points index in inliers

    }*/
            
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model  for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
    
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice],distanceTol);

	for(int id :  nearest)
	{
		if (!processed[id])
		clusterHelper(id, points, cluster, processed, tree, distanceTol);
	}

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters; 
	std::vector<bool> processed(points.size(),false);//initialised all points as boolean "0" not processed.

	int i=0;
	while (i < points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}
		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;

	}
 
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    /*typename pcl::search::KdTree<PointT>:: Ptr tree (new pcl::search::KdTree<PointT>);

    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);*/

    std::vector<std::vector<float>> points; 
    std::vector<float> point_each;

    KdTree* tree = new KdTree;
  
    for (int i=0; i<cloud->points.size(); i++) 
    {
	    point_each = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        points.push_back(point_each);
        tree->insert(points[i],i);
    }

	std::vector<std::vector<int>> cluster = euclideanCluster(points, tree, clusterTolerance);

    std::vector<pcl::PointIndices> clusterIndices;


    for(std::vector<int> clusters1 : cluster)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for(int index :clusters1)
        cloudCluster->points.push_back (cloud->points[index]);
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        if((clusters1.size() >= minSize) && (clusters1.size() <= maxSize))
        clusters.push_back(cloudCluster);

    }

    /*for(pcl::PointIndices getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for(int index :getIndices.indices)
        cloudCluster->points.push_back (cloud->points[index]);
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);

    }*/

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

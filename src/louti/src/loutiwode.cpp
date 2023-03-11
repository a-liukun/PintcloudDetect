#include <math.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/passthrough.h>  //直通滤波相关
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Geometry>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <stairs/preanalysis.h>
#include <stairs/regions.h>
#include <stairs/regiongrowing.h>
#include <stairs/voxSAC.h>
#include <stairs/splitmerge.h>
#include <stairs/planeshape.h>
#include <stairs/recognition.h>
#include <stairs/StairVector.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

using namespace std;
#include <pcl/filters/uniform_sampling.h>

//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("viewer"));


//回调函数
pcl::PointCloud<pcl::PointXYZ>::Ptr preorigin(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr cloud)
{
  pcl::fromROSMsg(*cloud, *preorigin);  //将得到的ros消息转换为点云指针preorigin
  //去除无效点
  std::vector<int> indices;
  //移除无效NaN点
  pcl::removeNaNFromPointCloud(*preorigin, *preorigin, indices);
   
    //体素网格下采样
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(preorigin);
    sor.setLeafSize(0.052f, 0.052f, 0.052f);
    sor.filter(*cloud_filtered);
    
   //均匀下采样
   // pcl::UniformSampling<pcl::PointXYZ> filter;		// 创建均匀采样对象
   // filter.setInputCloud(preorigin);					// 设置待采样点云
   // filter.setRadiusSearch(0.01f);					// 设置采样半径
   // filter.filter(*cloud_filtered);					// 执行均匀采样，结果保存在cloud_filtered中



/*if(argc < 3)
	{
		std::cerr << "Not enough arguments - " << argv[0] << " <input pcd> <output pcd>" << std::endl;
		return 1;
	}*/

// Loading input point cloud //

	int return_status;
  std::cout<<std::endl;
  std::cout<<std::endl;
	std::cout<<"Starting loading point cloud"<<std::endl;
	double loadS = pcl::getTime();

	PointCloudT::Ptr mainCloud;
	mainCloud.reset (new PointCloudT);
  mainCloud = cloud_filtered;

	//return_status = pcl::io::loadPCDFile (argv[1], *mainCloud);
	/*return_status = pcl::io::loadPCDFile (argv[1], *mainCloud);
	if (return_status != 0)
	{
		PCL_ERROR("Error reading point cloud %s\n", argv[1]);
		return -1;
	}
	double loadE = pcl::getTime();
	std::cout<<"Loading took: "<<loadE-loadS<<std::endl;  */

  // Starting preanalysis //

	  std::cout<<"Starting preanalysis"<<std::endl;
    double preAS = pcl::getTime();
    
    Preanalysis pre;
    NormalCloud::Ptr prepNomalCloud;
    prepNomalCloud.reset(new NormalCloud);
    PointCloudT floorPC;
    PointCloudC prepNorMap;

    pre.run(mainCloud, prepNomalCloud, prepNorMap, floorPC);


    double preAE = pcl::getTime();
    std::cout<<"Preanalysis took: "<<preAE-preAS<<std::endl;



// Starting segmentation //
    std::cout<<"Starting segmentation"<<std::endl;
    int mode = 0;
    double segS = pcl::getTime();
    regions segRegions;
    if(mode == 0)
    {
        std::cout<<"Using Region Growing algorihtm"<<std::endl;
        RegionGrowing reGrow;
        reGrow.setInputCloud(mainCloud);
        reGrow.setNormalCloud(prepNomalCloud);
        reGrow.run(segRegions);
    }
    if(mode == 1)
    {
        std::cout<<"Using Voxel SAC algorihtm"<<std::endl;
        voxSAC voxelSAC;
        voxelSAC.setInputCloud(mainCloud);
        voxelSAC.setNormalCloud(prepNomalCloud);
        voxelSAC.run(segRegions);
    }
    if(mode ==2)
    {
        std::cout<<"Using Split & Merge algorihtm"<<std::endl;
        splitMerge sam;
        sam.setInputCloud(mainCloud);
        sam.setNormalCloud(prepNomalCloud);
        sam.splitProcess();
        sam.mergeProcess(segRegions);
    }
    double segE = pcl::getTime();
    std::cout<<"Segmentation took: "<<segE-segS<<std::endl;

// Starting plane finder - plane extraction //

    std::cout<<"Starting plane finder"<<std::endl;

    double pfS = pcl::getTime();
    planeshape psProc;
    regions stairTreads;
    regions stairRisers;
    psProc.setInputRegions(segRegions);
    psProc.filterSc(stairTreads, stairRisers);

    double pfE = pcl::getTime();
    std::cout<<"Plane filter took: "<<pfE-pfS<<std::endl;

// Starting graph-based stair detection //
    std::cout<<"Starting graph-based detection"<<std::endl;
    StairVector detectedStairs;

    double refS = pcl::getTime();
    recognition stairDetect;
    stairDetect.setInputRegions(segRegions);
    stairDetect.setStairTreadRegions(stairTreads);
    stairDetect.setStairRiseRegions(stairRisers);
    //double refS = pcl::getTime();
    stairDetect.run(detectedStairs);  //若不下采样，会耗时
    double refE = pcl::getTime();

    std::cout<<"There are treads: "<<stairTreads.size()<<std::endl;
    std::cout<<"There are risers: "<<stairRisers.size()<<std::endl;

    std::cout<<"Refinement took: "<<refE-refS<<std::endl;
    std::cout<<"Total time  took: "<<refE-loadS<<std::endl;

// Printing out the results //

    bool colorByPart = true;

    PointCloudC resultCloud;

    bool addBackGround = true;
    if(addBackGround)
    {
    	for(size_t pointID = 0; pointID < mainCloud->size(); pointID ++)
    	{
    		PointTC backPoint;
    		backPoint.x = mainCloud->points[pointID].x;
    		backPoint.y = mainCloud->points[pointID].y;
    		backPoint.z = mainCloud->points[pointID].z;
    		backPoint.r=255;
    		backPoint.g=255;
    		backPoint.b=255;
    		resultCloud.push_back(backPoint);
    	}
    }

    std::cout<<"Detected stairways: "<<detectedStairs.size()<<std::endl;
    if(detectedStairs.size()>0)
    {
		for(int stairCoeffIdx =0; stairCoeffIdx < detectedStairs.size(); stairCoeffIdx++)
		{
			Stairs stairCoefficients;
			stairCoefficients = detectedStairs.at(stairCoeffIdx);

			float steigung = atan(stairCoefficients.dir[2] / sqrt(pow(stairCoefficients.dir[0],2) + pow(stairCoefficients.dir[1],2)));

			std::cout<<std::endl<<"Step depth:   "<<round(1000*sqrt(pow(stairCoefficients.dir[0],2) + pow(stairCoefficients.dir[1],2)))<<std::endl;
			std::cout<<"Step height:  "<<round(1000*stairCoefficients.dir[2])<<std::endl;
			std::cout<<"Step width:   "<<round(1000*stairCoefficients.width)<<std::endl;
			std::cout<<"Slope is:     "<<round(100*steigung/M_PI*180)<<std::endl;
			std::cout<<"Amount of stair parts: "<<stairCoefficients.size()<<std::endl<<std::endl;

			float stairAngle = atan2(stairCoefficients.dir[1],stairCoefficients.dir[0]);
			float xStairDist = stairCoefficients.pos[0];
			float yStairDist = stairCoefficients.pos[1];

			Eigen::Vector2f sepDist;
			sepDist[0] = cos(stairAngle) * xStairDist + sin(stairAngle) * yStairDist;
			sepDist[1] = - sin(stairAngle) * xStairDist + cos(stairAngle) * yStairDist;

	        std::cout<<"Dist in X is: "<<round(1000*(stairCoefficients.pos[0]))<<std::endl;
	        std::cout<<"Dist in Y is: "<<round(1000*stairCoefficients.pos[1])<<std::endl;

	        std::cout<<"Dist par is:  "<<round(1000*sepDist[0])<<std::endl;
	        std::cout<<"Dist ort is:  "<<round(1000*sepDist[1])<<std::endl;
			std::cout<<"Anchor point is: "<<stairCoefficients.anchPoint<<std::endl;

			std::cout<<"Angle is:     "<<round(100*atan2(stairCoefficients.dir[1],stairCoefficients.dir[0])/M_PI*180)<<std::endl;

			if(colorByPart)
				resultCloud += detectedStairs.getColoredParts(stairCoeffIdx);
			else
				resultCloud += detectedStairs.getColoredCloud(stairCoeffIdx);
		  }
    }
 /*
  //清空画图累计变量
  ch = 0;
  //去除无效点
  std::vector<int> indices;
  //移除无效NaN点
  pcl::removeNaNFromPointCloud(*preorigin, *preorigin, indices);
  if (preorigin->points.size() > 5000)
  {
    cout << "----------------正在进行一次楼梯检测------------------" << endl;
    flagloutiG = DetectGlobe(preorigin);
    return;
  }
  else
    return;
    */
}


/*
float r_w, r_x, r_y, r_z,p_x, p_y, p_z;
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
  if(!flagloutiG)
  {
    return;
  }
  p_x = odomMsg->pose.pose.position.x;
  p_y = odomMsg->pose.pose.position.y;
  p_z = odomMsg->pose.pose.position.z;
  r_x = odomMsg->pose.pose.orientation.x;
  r_y = odomMsg->pose.pose.orientation.y;
  r_z = odomMsg->pose.pose.orientation.z;
  r_w = odomMsg->pose.pose.orientation.w;
  Eigen::Quaterniond q2(r_w, r_x, r_y, r_z);
  Eigen::Vector3d t2(p_x, p_y, p_z);
  Eigen::Isometry3d T2w(q2);
  T2w.pretranslate(t2);

  //p1q为全局坐标
  Eigen::Vector3d p1q = T2w*p1;
  Eigen::Vector3d p2q = T2w*p2;
  Eigen::Vector3d p3q = T2w*p3;  
}
*/
//主函数
int main(int argc, char **argv)
{
  ros::Publisher marker_pub;
  ros::init(argc, argv, "detect_louti");
  ros::NodeHandle nh;
  //ros::Subscriber sub = nh.subscribe("/hesai/pandar", 1, chatterCallback);
  //ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, chatterCallback);
  ros::Subscriber sub = nh.subscribe("/rslidar_points", 1, chatterCallback);
  // ros::Publisher pcl_pub =nh.advertise<sensor_msgs::PointCloud2>("pcl_output1", 1);
 
  //ros::Subscriber subcoordinate = nh.subscribe("lio_sam/mapping/odometry_incremental", 1, odometryHandler);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker_louti", 1);
  //定义循环执行的频率为10hz，即一秒执行10次，一次执行整个消息队列长度的回调函数
  //也就是一秒执行10帧，如果发布者发布消息的频率大于10hz将丢包
  ros::Rate loop_rate(10);

  //检测一次楼梯
  // while(ros::ok( )&&!flagloutiG)

  //持续检测楼梯
  while (ros::ok())
  {
    //执行一次回调函数
    ros::spinOnce();
    /*
    //刷新画板
    if (STEP >= (char)64 && STEP <= 'Y')
    {
      viewer1->spinOnce(0.000000000001);
      // 移除当前所有点云
      viewer1->removeAllPointClouds();
      //移除一片点云，参数为点云的名字
      // viewer->removePointCloud("cloud");
      //移除窗口中所有形状
      viewer1->removeAllShapes();
      // viewer1->remove
    }
    */
    //休眠
    loop_rate.sleep();
  }
  return 0;
}
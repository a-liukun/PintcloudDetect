#include <math.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
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

//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"

using namespace std;

#define NR 0.1
#define PI 3.1415
#define DIR "/home/huo/"

char ch = 'a';
int no = 0;
char STEP = 'Z';
clock_t read_start, start, cal_start, now;
bool flaggouG=false;
Eigen::Vector3d p1; //沟区域质心
Eigen::Vector3d p2; //沟区域质心

ros::Publisher marker_pub;
uint32_t shape = visualization_msgs::Marker::CUBE;    //设置发送形状为立方体

// pcl::visualization::PCLVisualizer viewer1("segmention");
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("viewer"));

int no1 = 0;

void showoushijulei(std::vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
  for (int j = 0; j < cluster_indices.size(); j++)
  {
    //研究当前索引
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = cluster_indices[j].indices.begin();
         pit != cluster_indices[j].indices.end(); ++pit)
      cloud_cluster->points.push_back(
          cloud_ptr->points[*pit]);  //每次创建一个新的点云数据集，并且将所有当前聚类的点写入到该点云数据集中。
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points."
              << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";

    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> cloud_in_color_h(
        cloud_ptr);  //赋予显示点云的颜色,随机
    viewer1->addPointCloud(cloud_cluster, cloud_in_color_h, std::to_string(ch++));
  }
}

void showdanyunsuoyin(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1(new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = inliers->indices.begin(); pit != inliers->indices.end(); ++pit)
    cloud_cluster1->points.push_back(
        cloud_cluster->points[*pit]);  
  cloud_cluster1->width = cloud_cluster->points.size();
  cloud_cluster1->height = 1;
  cloud_cluster1->is_dense = true;

  pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> cloud_in_color_h(
      cloud_cluster1); 
  viewer1->addPointCloud(cloud_cluster1, cloud_in_color_h, to_string(ch++));
}

void showdianyunred(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_ptr, 250, 0, 0);
  viewer1->addPointCloud(cloud_ptr, color, "cloudred" + to_string(ch++));
}

void showdianyungreen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_ptr, 0, 250, 0);
  viewer1->addPointCloud(cloud_ptr, color, "cloudgreen" + to_string(ch++));
}

void showdianyunblue(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_ptr, 0, 0, 250);
  viewer1->addPointCloud(cloud_ptr, color, "cloudgreen" + to_string(ch++));
}

void tiquoushijulei(std::vector<pcl::PointIndices> clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr orgin,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &outputs)
{
  for (int j = 0; j < clusters.size(); j++)
  {
    for (std::vector<int>::const_iterator pit = clusters[j].indices.begin(); pit != clusters[j].indices.end(); ++pit)
      outputs->points.push_back(orgin->points[*pit]);  //ÿ�δ���һ���µĵ������ݼ������ҽ����е�ǰ����ĵ�д�뵽�õ������ݼ��С�
    outputs->width = outputs->points.size();
    outputs->height = 1;
    outputs->is_dense = true;
  }
}

void Directfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &origin, double a,double b)
{
  // a为x轴滤波参数，b为y轴滤波参数
  pcl::PassThrough<pcl::PointXYZ> pass;  //创建滤波器对象
  pass.setInputCloud(origin);            //设置待滤波的点云
  pass.setFilterFieldName("z");          //设置在x轴方向上进行滤波
  pass.setFilterLimits(a, b);           //设置滤波范围
  pass.setFilterLimitsNegative(false);   //保留范围
  pass.filter(*origin);                  //滤波并存储
}

void removecenter(pcl::PointCloud<pcl::PointXYZ>::Ptr &orgin)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud = orgin;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;  //创建滤波器对象
  pass.setInputCloud(cloud);             //设置待滤波的点云
  pass.setFilterFieldName("y");          //设置在x轴方向上进行滤波
  pass.setFilterLimits(0.2, 0.4);        //设置滤波范围
  pass.setFilterLimitsNegative(true);    //范围内
  pass.filter(*cloud);                   //滤波并存储
  // showdianyunred(cloud);
}

void normalems(pcl::PointCloud<pcl::PointXYZ>::Ptr orgin, pcl::PointCloud<pcl::PointXYZ>::Ptr &outputs)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
  cout << "->加载点云个数：" << orgin->points.size() << endl;
  //===============================================================
  pcl::PassThrough<pcl::PointXYZ> pass;  //创建滤波器对象
  pass.setInputCloud(orgin);             //设置待滤波的点云
  pass.setFilterFieldName("z");          //设置在x轴方向上进行滤波
  pass.setFilterLimits(-10, -0.9);       //设置滤波范围
  pass.setFilterLimitsNegative(false);   //潜在地面底部,false为保留范围之内
  pass.filter(*cloud);                   //滤波并存储
  cout << "cloud  " << cloud->points.size() << endl;
  pass.setFilterLimitsNegative(true);  //地面以外，true为保留范围之外
  pass.filter(*cloud1);                //滤波并存储
  cout << "cloud1  " << cloud1->points.size() << endl;

  clock_t time;
  time = clock();
  //--------------------------- 法线估计 ---------------------------
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;  //创建法线估计对象
  ne.setInputCloud(cloud);                               //设置法线估计输入点云
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());  //创建一个空的kdtree
  ne.setSearchMethod(tree);  //将空kdtree传递给法线估计对象 ne
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);  //法向量计算结果
  ne.setKSearch(10);                                                            //设置K近邻的个数
  // ne.setRadiusSearch(0.05);	//设置半径邻域的大小，两种方式二选一
  ne.setViewPoint(0, 0, 1);  //设置视点向量，默认0向量(0,0,0)，没有方向
  ne.compute(*normals);      //执行法线估计，并将结果保存到normals中
  //===============================================================
  cout << "->法线估计用时：" << (double)(clock() - time) / CLOCKS_PER_SEC << " s" << endl;
  // 显示点云的法向量
  // viewer1.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals,
  // 20, 0.5, "normals");
  // //每十个点显示一个法线，长度为0.05

  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aim = cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
  boundEst.setInputCloud(aim);
  boundEst.setInputNormals(normals);
  boundEst.setRadiusSearch(0.1);
  boundEst.setAngleThreshold(M_PI / 1.2);  //角度越大删的点越少保留的越多。角度越小删的点越多保留的也越少
  boundEst.setSearchMethod(tree);
  boundEst.compute(boundaries);
  for (int i = aim->points.size() - 1; i >= 0; i--)
    if (!(boundaries[i].boundary_point > 0))
      result->push_back(aim->points[i]);
  cout << "边缘移除前" << aim->points.size() << endl;
  cout << "边缘移除后" << result->points.size() << endl;
  cloud1 = (*result + *cloud1).makeShared();
  cout << "cloud1  " << cloud1->points.size() << endl;
  // showdianyunred(orgin);
  // //cloud1为删除地面后的点云
  // showdianyungreen(cloud1);
  outputs = cloud1;
  return;

}

void Removeplane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr)
{
  int q = 1;
  //去除墙面
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clusterorgin(new pcl::PointCloud<pcl::PointXYZ>);  //保存原始点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1(new pcl::PointCloud<pcl::PointXYZ>);  //保存删除墙壁立面后的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster2(new pcl::PointCloud<pcl::PointXYZ>);  //保存墙壁立面点云
  cloud_clusterorgin = cloud_ptr;
  do
  {
    // pcl::visualization::PCLVisualizer viewer("segmention");
    clock_t time=clock();
    ne.setSearchMethod(tree1);
    ne.setInputCloud(cloud_ptr);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);  //法向量与轴平行，平面与轴垂直
    seg.setMethodType(pcl::SAC_PROSAC);
    //   const static int SAC_RANSAC  = 0;
    // const static int SAC_PROSAC  = 6; 优先采用,迭代速度较快

    //设置轴为y轴
    seg.setAxis(Eigen::Vector3f(1, 0, 0));
    //该角度为平面与轴的多方位夹角，结果为整个所设置的角度中内点最多的
    seg.setEpsAngle(0.785);       //  如果要找出很垂直与y轴的立面角度设置为0.2，最大不要超过0.785
    seg.setMaxIterations(100);  //设置为500次
    seg.setDistanceThreshold(0.1);  //单位米，点到模型的距离
    seg.setInputCloud(cloud_ptr);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);  //删除墙壁立面,剩下的点云
    extract.filter(*cloud_cluster1);

    pcl::ExtractIndices<pcl::PointXYZ> extract1;
    extract1.setInputCloud(cloud_ptr);
    extract1.setIndices(inliers);
    extract1.setNegative(false);       //保留墙壁
    extract1.filter(*cloud_cluster2);  //用它的大小来控制循环结束

    cloud_ptr = cloud_cluster1;  //更新一下cloud_ptr

    // viewer.addCoordinateSystem(5);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud_cluster1, 250, 0, 0);
    // viewer.addPointCloud(cloud_cluster1, color1, "cloudred" + to_string(ch++));

    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud_cluster2, 0, 250, 0);
    // viewer.addPointCloud(cloud_cluster2, color2, "cloudgreen" + to_string(ch++));

    cout << "第" << q << "次循环拟合立面平云点的个数" << cloud_cluster2->points.size() << endl;
    cout << "第" << q++ << "次循环拟合立面SAC用时：" << (double)(clock() - time) / CLOCKS_PER_SEC << " s" << endl;
  } while (cloud_cluster2->points.size() > 2000);

  if (STEP == 'B')  //去除立面点云
  {
    showdianyungreen(cloud_clusterorgin);
    showdianyunred(cloud_ptr);
  }
}

void RorFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr)
{
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin(new pcl::PointCloud<pcl::PointXYZ>);  //保存原始点云
  // cloud_origin=cloud_ptr;
  // cloud_origin=(*cloud_ptr).makeShared();
  if (STEP == 'C')  //统计滤波
  {
    cout<<"统计滤波前cloud_ptr的点云个数"<<cloud_ptr->points.size()<<endl;
    showdianyungreen(cloud_ptr);
  }
  clock_t time=clock();
  // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
  //个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
  sor.setInputCloud (cloud_ptr);                           //设置待滤波的点云
  sor.setMeanK (500);                               //设置在进行统计时考虑查询点临近点数
  sor.setStddevMulThresh (0.1);                      //设置判断是否为离群点的阀值
  sor.filter (*cloud_ptr);                    //存储
  cout << "统计滤波用时：" << (double)(clock() - time) / CLOCKS_PER_SEC << " s" << endl;
  if (STEP == 'C')  //去除立面点云
  {
    cout<<"统计滤波后cloud_ptr的点云个数"<<cloud_ptr->points.size()<<endl;
    showdianyunred(cloud_ptr);
  }
}

void Radiusoutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr, float VX0, int minnum)
{
  if (STEP == 'D')  //统计滤波
  {
    cout<<"半径滤波前cloud_ptr的点云个数"<<cloud_ptr->points.size()<<endl;
    showdianyungreen(cloud_ptr);
  }
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_ptr);
  sor.setRadiusSearch(VX0);
  sor.setMinNeighborsInRadius(minnum);
  sor.setNegative(false);
  sor.filter(*cloud_ptr);
  if (STEP == 'D')  //统计滤波
  {
    cout << "半径滤波后"
	   << " " << cloud_ptr->points.size() << endl;
    showdianyunred(cloud_ptr);
  }
}

void extractground(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr)
{
  //去除墙面
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clusterorgin(new pcl::PointCloud<pcl::PointXYZ>);  //保存原始点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1(new pcl::PointCloud<pcl::PointXYZ>);  //保存删除墙壁立面后的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster2(new pcl::PointCloud<pcl::PointXYZ>);  //保存墙壁立面点云
  cloud_clusterorgin = cloud_ptr;
  int q = 1;
  do
  {
    clock_t time=clock();
    ne.setSearchMethod(tree1);
    ne.setInputCloud(cloud_ptr);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);  //法向量与轴平行，平面与轴垂直
    seg.setMethodType(pcl::SAC_PROSAC);
    //   const static int SAC_RANSAC  = 0;
    // const static int SAC_PROSAC  = 6; 优先采用,迭代速度较快

    //设置轴为z轴
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    //该角度为平面与轴的多方位夹角，结果为整个所设置的角度中内点最多的
    seg.setEpsAngle(0.2);       //  如果要找出很垂直与y轴的立面角度设置为0.2，最大不要超过0.785
    seg.setMaxIterations(100);  //设置为500次
    seg.setDistanceThreshold(0.1);  //单位米，点到模型的距离
    seg.setInputCloud(cloud_ptr);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);  //删除墙壁立面,剩下的点云
    extract.filter(*cloud_cluster1);

    pcl::ExtractIndices<pcl::PointXYZ> extract1;
    extract1.setInputCloud(cloud_ptr);
    extract1.setIndices(inliers);
    extract1.setNegative(false);       //保留墙壁
    extract1.filter(*cloud_cluster2);  //用它的大小来控制循环结束

    cloud_ptr = cloud_cluster1;  //更新一下cloud_ptr

    Eigen::Vector4f Centroid;  //质心
    pcl::compute3DCentroid(*cloud_cluster2, Centroid);
    cout<<endl;
    if(q==1)
    {
      p1=Eigen::Vector3d(Centroid.x(),Centroid.y(),Centroid.z());
    }
    if(q==2&&cloud_cluster2->points.size() >= 1000)
    {
      p2=Eigen::Vector3d(Centroid.x(),Centroid.y(),Centroid.z());
    }
    
    cout<<"地面的平均高度为"<<Centroid[2]<<endl;
    cout<<"地面的法向量为"<<*coefficients<<endl;

    cout << "第" << q << "次循环拟合立面平云点的个数" << cloud_cluster2->points.size() << endl;
    cout << "第" << q << "次循环拟合地面SAC用时:" << (double)(clock() - time) / CLOCKS_PER_SEC << " s" << endl;
    cout<<endl;
    cloud_ptr = cloud_cluster1;  //更新一下cloud_ptr
    q++;
  } while (q<=2);
  if (STEP == 'B')  
  {
    showdianyungreen(cloud_clusterorgin);
    showdianyunred(cloud_ptr);
  }

  if(p1(2)>p2(2))
  {
      p1=p2;
  }
  }

//extractgou未使用
//如果为x轴正对着沟（x轴与沟的长轴垂直），右侧为y轴，那么拟合沟前侧面垂直的轴为（-1，0，1），后侧面遮挡
//如果为x轴与沟的长轴平行，右侧为y轴，那么拟合沟沟右侧面垂直的轴为（0，-1，1），左侧垂直的轴为（0，1，1）
void extractgou(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr)
{
  int q = 1;
  //去除墙面
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clusterorgin(new pcl::PointCloud<pcl::PointXYZ>);  //保存原始点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1(new pcl::PointCloud<pcl::PointXYZ>);  //保存删除墙壁立面后的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster2(new pcl::PointCloud<pcl::PointXYZ>);  //保存墙壁立面点云
  cloud_clusterorgin = cloud_ptr;
  do
  {
    // pcl::visualization::PCLVisualizer viewer("segmention");
    clock_t time=clock();
    ne.setSearchMethod(tree1);
    ne.setInputCloud(cloud_ptr);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);  //法向量与轴平行，平面与轴垂直
    seg.setMethodType(pcl::SAC_PROSAC);
    //   const static int SAC_RANSAC  = 0;
    // const static int SAC_PROSAC  = 6; 优先采用,迭代速度较快

    //设置轴为y轴
    seg.setAxis(Eigen::Vector3f(0, -1, 1));
    //该角度为平面与轴的多方位夹角，结果为整个所设置的角度中内点最多的
    seg.setEpsAngle(0.785);       //  如果要找出很垂直与y轴的立面角度设置为0.2，最大不要超过0.785
    seg.setMaxIterations(100);  //设置为500次
    seg.setDistanceThreshold(0.2);  //单位米，点到模型的距离
    seg.setInputCloud(cloud_ptr);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);  //删除墙壁立面,剩下的点云
    extract.filter(*cloud_cluster1);

    pcl::ExtractIndices<pcl::PointXYZ> extract1;
    extract1.setInputCloud(cloud_ptr);
    extract1.setIndices(inliers);
    extract1.setNegative(false);       //保留墙壁
    extract1.filter(*cloud_cluster2);  //用它的大小来控制循环结束

    cloud_ptr = cloud_cluster1;  //更新一下cloud_ptr

    Eigen::Vector4f Centroid;  //质心
    pcl::compute3DCentroid(*cloud_cluster2, Centroid);
    cout<<endl;

    if(q==1)
    {
    cout<<"沟面的平均高度为"<<Centroid[2]<<endl;
    cout<<"沟面的法向量为"<<*coefficients<<endl;
    }

    // viewer.addCoordinateSystem(5);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud_cluster1, 250, 0, 0);
    // viewer.addPointCloud(cloud_cluster1, color1, "cloudred" + to_string(ch++));

    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud_cluster2, 0, 250, 0);
    // viewer.addPointCloud(cloud_cluster2, color2, "cloudgreen" + to_string(ch++));

    cout << "第" << q+1 << "次循环拟合平面平云点的个数" << cloud_cluster2->points.size() << endl;
    cout << "第" << q+1 << "次循环拟合平面SAC用时：" << (double)(clock() - time) / CLOCKS_PER_SEC << " s" << endl;
    cout<<endl;
    q++;
  // } while (cloud_cluster2->points.size() > 2000);
  } while (q<=1);
  if (STEP == 'C')  //去除立面点云
  {
    showdianyungreen(cloud_clusterorgin);
    showdianyunred(cloud_ptr);
  }
}

bool Detectgou(pcl::PointCloud<pcl::PointXYZ>::Ptr origin)
{
  Directfilter(origin,-2,-0.5);
  if(origin->points.size()<=1000)
  {
    return false;
  }
  extractground(origin);
  cout<<"groundz的高度:"<<p1(2)<<endl;
  cout<<"gouz的高度:"<<p2(2)<<endl;
  return fabs(p1(2)-p2(2))>0.3&&p1(2)!=0&&p2(2)!=0?true:false;
}

//回调函数
pcl::PointCloud<pcl::PointXYZ>::Ptr preorigin(new pcl::PointCloud<pcl::PointXYZ>());
void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr cloud)
{
  pcl::fromROSMsg(*cloud, *preorigin);  //将得到的ros消息转换为点云指针preorigin
  //清空画图累计变量
  ch = 0;
  //去除无效点
  std::vector<int> indices;
  //移除无效NaN点
  pcl::removeNaNFromPointCloud(*preorigin, *preorigin, indices);
  if (preorigin->points.size() > 5000)
  {
    cout << "----------------正在进行一次沟检测------------------" << endl;
    flaggouG = Detectgou(preorigin);
    return;
  }
  else
    return;
}

void DrawCube(Eigen::Vector3d p1,ros::Time s)
{
    visualization_msgs::Marker marker;
    //发送质心坐标系到rviz
    // 设置帧 ID和时间戳
    marker.header.frame_id = "map";
    marker.header.stamp = s;
    // 设置该标记的命名空间和ID，ID应该是独一无二的
    // 具有相同命名空间和ID的标记将会覆盖前一个
    marker.ns = "basic_shapes_gou";
      int i=0;
      marker.id = i;
      cout<<"marker:"<<marker.id<<endl;
      marker.type = shape;

      // 设置标记行为：ADD（添 加），DELETE（删 除）
      marker.action = visualization_msgs::Marker::ADD;

      //设置标记位姿。
      marker.pose.position.x = p1(0);
      marker.pose.position.y = p1(1);
      marker.pose.position.z = p1(2);
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // 设置标记的比例，所有方向上尺度1表示1米
      // marker.scale.x = p3(0)-p2(0);
      // marker.scale.y = p3(1)-p2(1);
      // marker.scale.z = p3(2)-p2(2);

      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z =1;

      //设置标记颜色，确保alpha（不透明度）值不为0
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1;
      marker.lifetime = ros::Duration();
      marker_pub.publish(marker);
      cout << "发送第" << i << "个沟marker" << endl;
}

float r_w, r_x, r_y, r_z,p_x, p_y, p_z;
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
  if(!flaggouG)
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
  DrawCube(p1q,odomMsg->header.stamp);
}

//主函数
int main(int argc, char **argv)
{
  //设置背景色为白色
  viewer1->setBackgroundColor(255, 255, 255);
  viewer1->addCoordinateSystem(0.5);
  //设置视角
  // viewer1->initCameraParameters();
  if (argv[1] != NULL)
  {
    STEP = argv[1][0];
  }
  ros::init(argc, argv, "detect_gou");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/hesai/pandar", 1, chatterCallback);
  //ros::Subscriber sub = nh.subscribe("/partMap", 1, chatterCallback);
  // ros::Publisher pcl_pub =nh.advertise<sensor_msgs::PointCloud2>("pcl_output1", 1);

  ros::Subscriber subcoordinate = nh.subscribe("lio_sam/mapping/odometry_incremental", 1, odometryHandler);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker_gou", 1);
  //定义循环执行的频率为10hz，即一秒执行10次，一次执行整个消息队列长度的回调函数
  //也就是一秒执行10帧，如果发布者发布消息的频率大于10hz将丢包
  ros::Rate loop_rate(100);

  //检测一次楼梯
  // while(ros::ok( )&&!flaggouG)

  //持续检测楼梯
  while (ros::ok())
  {
    //执行一次回调函数
    ros::spinOnce();
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
    //休眠
    loop_rate.sleep();
  }
  return 0;
}
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

//#include <Open3D/IO/IO.h>
//#include <Open3D/Geometry/PointCloud.h>
//#include <Open3D/Geometry/KDTreeFlann.h>

using namespace std;

char ch = 'a';
char STEP = 'Z';
bool flagpoG = false;
Eigen::Vector3d p1; //楼梯质心
Eigen::Vector3d p2; //框子min点
Eigen::Vector3d p3; //框子max点
ros::Publisher marker_pub;
uint32_t shape = visualization_msgs::Marker::CUBE;    //设置发送形状为立方体

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("viewer"));

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

void showdianyunset(vector<pcl::PointCloud<pcl::PointXYZ>> v_clouds)
{
  for (int j = 0; j < v_clouds.size(); j++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = v_clouds[j].makeShared();
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> cloud_in_color_h(
        cloudPtr);  //赋予显示点云的颜色,随机
    viewer1->addPointCloud(cloudPtr, cloud_in_color_h, to_string(ch++));
  }
}

void showdianyunred(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_ptr, 255, 0, 0);
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
  viewer1->addPointCloud(cloud_ptr, color, "cloudbule" + to_string(ch++));
}

void ReadPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr &origin, string s)
{
  cout << "要打开的pcd文件名为" << s << ".pcd" << endl;
  pcl::io::loadPCDFile(s + ".pcd", *origin);
  cout << "Read " << origin->points.size() << endl;
  std::vector<int> indices;
  //移除无效NaN点
  pcl::removeNaNFromPointCloud(*origin, *origin, indices);  // inf
}

void Directfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &origin, double a, double b)
{
  // a为x轴滤波参数，b为y轴滤波参数
  pcl::PassThrough<pcl::PointXYZ> pass;  //创建滤波器对象
  pass.setInputCloud(origin);            //设置待滤波的点云
  pass.setFilterFieldName("x");          //设置在x轴方向上进行滤波
  pass.setFilterLimits(-a, a);           //设置滤波范围
  pass.setFilterLimitsNegative(false);   //保留范围
  pass.filter(*origin);                  //滤波并存储
  pass.setInputCloud(origin);            //设置待滤波的点云
  pass.setFilterFieldName("y");          //设置在y轴方向上进行滤波
  pass.setFilterLimits(-b, -0.5);        //设置滤波范围
  pass.setFilterLimitsNegative(false);   //保留范围
  pass.filter(*origin);
  cout << "直通滤波"
       << " " << origin->points.size() << endl;
}

void Upsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_fl)
{
  clock_t begin = clock();
  //   pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
  //    filter.setInputCloud(cloud_ptr);    //建立搜索对象
  //    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
  //    filter.setSearchMethod(kdtree);    //设置搜索邻域的半径为3cm
  //    filter.setSearchRadius(0.02);      //越大越慢
  //    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
  //    filter.setUpsamplingRadius(0.03);    // 采样的半径不能小于kd树半径
  //    filter.setUpsamplingStepSize(0.03);  // 采样的步数是与上面保持一致
  //    filter.process(*cloud_fl);

  //   pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
  //    filter.setInputCloud(cloud_ptr);    //建立搜索对象
  //    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
  //    filter.setSearchMethod(kdtree);    //设置搜索邻域的半径为3cm
  //    filter.setSearchRadius(0.02);      //越大越慢
  //    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::RANDOM_UNIFORM_DENSITY);
  //    filter.setPointDensity(100);
  //    filter.process(*cloud_fl);
  clock_t end = clock();

  cout << "增采样Running time: " << ((double)(end) - (double)(begin)) / CLOCKS_PER_SEC << "s" << endl;
}

void Voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr, float VX0)
{
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud_ptr);
  vg.setLeafSize(VX0, VX0, VX0);  //单位是米
  vg.filter(*cloud_ptr);
  cout << "体素滤波"
       << " " << cloud_ptr->points.size() << endl;
}

void Radiusoutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr, float VX0, int minnum)
{
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_ptr);
  sor.setRadiusSearch(VX0);
  sor.setMinNeighborsInRadius(minnum);
  sor.setNegative(false);
  sor.filter(*cloud_ptr);
  cout << "半径滤波"
       << " " << cloud_ptr->points.size() << endl;
}

bool customCondition(const pcl::PointXYZ &seedPoint, const pcl::PointXYZ &candidatePoint, float squaredDistance)
{
  //该条件欧式聚类调用效率很低
  //如果为x则聚类结果垂直于x轴
  //如果为y则聚类结果垂直于y轴
  return candidatePoint.y > seedPoint.y ? true : false;
}

void ConditionalEuclideanclustering(pcl::PointCloud<pcl::PointXYZ>::Ptr &origin,
                                    std::vector<pcl::PointIndices> &clusters)
{
  pcl::ConditionalEuclideanClustering<pcl::PointXYZ> ec;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  ec.setClusterTolerance(0.2);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(10000);
  ec.setConditionFunction(&customCondition);
  ec.setInputCloud(origin);
  ec.segment(clusters);  //得到每一片楼梯立面的点云索引
}

void RANSACcylinder1(pcl::PointCloud<pcl::PointXYZ>::Ptr origin)
{
  //优先采用此分割办法
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

  // Datasets
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers(new pcl::PointIndices);

  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
  pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);

  clicked_points_3d = origin;
  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(clicked_points_3d);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
  /*
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg
  可以用来分割，全部都基于法线
  SACMODEL_CYLINDER 需要设置半径，NormalDistanceWeight(法线距离权重)，该权重与距离成正比，与角度成反比。轴和角度

  SACMODEL_NORMAL_PLANE 需要设置NormalDistanceWeight(法线距离权重)也可以不设置   拟合率高常用到
  SACMODEL_NORMAL_PARALLEL_PLANE
  需要设置NormalDistanceWeight(法线距离权重)0.1即可也可以不设置，setDistanceFromOrigins(模型到原点的距离)也可以不设置
  轴和角度，法向量垂直于轴  该方法常用到
  一般只需要设置轴和角度

  SACMODEL_CONE 需要设置MinMaxOpeningAngle，NormalDistanceWeight，轴和角度
  //该函数配合，当用户指定模型为圆锥模型时，设置圆锥模型锥角的最小值与最大值，作为估计时的取值范围。
  SACMODEL_NORMAL_SPHERE 需要设置半径，NormalDistanceWeight(法线距离权重)
  以及包括pcl::SACSegmentation<pcl::PointXYZ> seg
  */

  clock_t begin = clock();

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);  //法向量与轴平行，平面与轴垂直
  seg.setMethodType(6);

  //   const static int SAC_RANSAC  = 0;
  // const static int SAC_PROSAC  = 6; 优先采用,迭代速度较快
  seg.setAxis(Eigen::Vector3f(1, 0, 0));
  //该角度为平面与轴的多方位夹角，结果为整个所设置的角度中内点最多的
  seg.setEpsAngle(0.1);       //  如果要找出很垂直与轴的立面角度设置为0.2，最大不要超过0.785
  seg.setMaxIterations(500);  //设置为500次
  seg.setDistanceThreshold(0.2);  //单位米，点到模型的距离

  /*
  seg.setRadiusLimits(0, 5); //单位米  设置半径的限制
  seg.setNormalDistanceWeight(0.1); //点法线和平面法线角度的权重，默认选择0.1即可，影响不显著
  seg.setDistanceFromOrigin(10);//设置模型到原点的距离
  */

  /*
  使用总结：
  一.当对整体环境滤除立面时，如果包含了部分坡面，则稍减小0.785，分为两大部分：
  ①将轴设置为x轴，角度为0.785即为45度（-45，+45）90度，旋转无限大的平面角度乘2，即为180度。
  ②将轴设置为y轴，角度为0.785即为45度（-45，+45）同理，合起来为360度。
  二.当对整体环境识别坡面时，将轴设置为z轴，角度为要识别坡面的弧度制，
  当为地面时，角度约为0.1~0.2 6度~10度左右，
  一般的坡面角度不会超过0.785。

  综上所述，角度范围为（0.1，0.785）。
  */
  seg.setInputCloud(clicked_points_3d);
  seg.setInputNormals(cloud_normals);
  seg.segment(*inliers, *coefficients);

  std::cerr << "coefficients: " << *coefficients << std::endl;
  cout << "--------------------------------" << endl;
  clock_t end = clock();
  cout << "Running time: " << ((double)(end) - (double)(begin)) / CLOCKS_PER_SEC << "s" << endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_cluster = clicked_points_3d;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_cluster);
  extract.setIndices(inliers);
  extract.setNegative(true);  //删除墙壁,剩下的点云
  extract.filter(*cloud_cluster1);
  showdianyunred(cloud_cluster1);

  pcl::ExtractIndices<pcl::PointXYZ> extract1;
  extract1.setInputCloud(cloud_cluster);
  extract1.setIndices(inliers);
  extract1.setNegative(false);  //保留墙壁
  extract1.filter(*cloud_cluster2);
  showdianyungreen(cloud_cluster2);
}

void RANSACcylinder2(pcl::PointCloud<pcl::PointXYZ>::Ptr origin)
{
  /*
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  可以用来分割
  SACMODEL_PLANE
  SACMODEL_LINE
  SACMODEL_STICK 需要设置半径
  SACMODEL_CIRCLE2D 需要设置半径
  SACMODEL_CIRCLE3D 需要设置半径
  SACMODEL_SPHERE 需要设置半径
  SACMODEL_PARALLEL_LINE 需要设置轴和角度  提取立面
  SACMODEL_PERPENDICULAR_PLANE 需要设置轴和角度  提取立面,轴设置为x或者y轴，角度设置为0.5PI  常用到
  SACMODEL_PARALLEL_PLANE 需要设置轴和角度
  以外的将会报错 No valid model given!
  */
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // inliers表示误差能容忍的点 记录的是点云的序号
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // 创建一个分割器
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。选择为true
  seg.setOptimizeCoefficients(true);
  // Mandatory-设置目标几何形状
  //  seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);  //拟合平面
  seg.setModelType(pcl::SACMODEL_PLANE);  //拟合平面
                                          // seg.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
  // seg.setEpsAngle(0.1);

  string s = to_string(seg.getAxis()(0)).substr(0, 1) + " ";
  s += to_string(seg.getAxis()(1)).substr(0, 1) + " ";
  s += to_string(seg.getAxis()(2)).substr(0, 1) + " ";
  viewer1->addText(s, 10, 10, 20, 1, 1, 1, "text");  //其中第三个10为字体的大小
                                                     // model.setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
  // model.setEpsAngle (pcl::deg2rad (15));
  //分割方法：随机采样法
  seg.setMethodType(pcl::SAC_RANSAC);
  //设置误差容忍范围，也就是阈值
  seg.setMaxIterations(200);
  seg.setDistanceThreshold(0.1);
  //输入点云
  seg.setInputCloud(origin);
  //分割点云
  seg.segment(*inliers, *coefficients);

  // cout<<"轴"<<endl<<seg.getAxis()<<endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_cluster = origin;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster2(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // extract.setInputCloud(cloud_cluster);
  // extract.setIndices(inliers);
  // extract.setNegative(true);  //删除模型,剩下的点云
  // extract.filter(*cloud_cluster1);
  // showdianyunred(cloud_cluster1);

  pcl::ExtractIndices<pcl::PointXYZ> extract1;
  extract1.setInputCloud(cloud_cluster);
  extract1.setIndices(inliers);
  extract1.setNegative(false);  //保留模型
  extract1.filter(*cloud_cluster2);
  showdianyunblue(cloud_cluster2);
}

void print_cube(pcl::PointCloud<pcl::PointXYZ>::Ptr origin)
{
  // cube
  pcl::ModelCoefficients tmp;
  tmp.values.push_back(0.1);
  tmp.values.push_back(0.1);
  tmp.values.push_back(0.1);
  tmp.values.push_back(1);
  tmp.values.push_back(1);
  tmp.values.push_back(1);
  tmp.values.push_back(1);
  tmp.values.push_back(10);
  tmp.values.push_back(10);
  tmp.values.push_back(10);
  // viewer1->addCube(tmp, "cube");
  // viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
  // 									  pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
  // 									  "cube");  //按框渲染
  // viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
  // 									  pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
  // 									  "cube");  //按面渲染
  // viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
  // 									  pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS,
  // 									  "cube");  //按点渲染
  // viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0,
  //									  "cube");
  // viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1,"cube");

  //二维文字，以窗口左下角为坐标原点,字体附着在窗口原点
  string str = "hello world";
  // double型的rgb都归一化处理
  viewer1->addText(str, 10, 10, 100, 0, 0, 0, "text");  //其中100为字体的大小

  //三维文字
  // pcl::visualization::PCLVisualizer::addText3D (
  // const std::string &text,
  // const PointT& position,
  // double textScale,  字体大小默认为1
  // double r,
  // double g,
  // double b,
  // const std::string &id,
  // int viewport)
  pcl::PointXYZ position0(10, 0, 0);
  string str0 = "hello world0";
  // double型的rgb都归一化处理，默认为1即白色
  viewer1->addText3D(str0, position0, 1, 0, 0, 0);

  pcl::PointXYZ position1(0, 10, 0);
  string str1 = "hello world1";
  // double型的rgb都归一化处理，默认为1即白色
  viewer1->addText3D(str1, position1, 1, 0, 0, 0);

  pcl::PointXYZ position2(0, 0, 10);
  string str2 = "hello world2";
  // double型的rgb都归一化处理，默认为1即白色
  viewer1->addText3D(str2, position2, 1, 0, 0, 0);

  // viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE,10,"text");
  //点云
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(origin, 255, 0, 0);
  // viewer1->addPointCloud(origin, color, "transformCloud");
  // viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"transformCloud");
  // viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1,"transformCloud");
  // viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT,
  // 								  pcl::visualization::PCL_VISUALIZER_LUT_BLUE2RED,
  // 								  "transformCloud");
  // enum RenderingProperties   //字体，点云和形状都可以设置
  // {
  //   PCL_VISUALIZER_POINT_SIZE,            /**< integer starting from 1 */   设置点云点的大小
  //   PCL_VISUALIZER_OPACITY,               /**< Float going from 0.0 (transparent) to 1.0 (opaque) */
  //   设置透明性0为透明 PCL_VISUALIZER_LINE_WIDTH,            /**< Integer starting from 1 */
  //   设置线的宽度,适用于形状的属性 PCL_VISUALIZER_FONT_SIZE, 设置字体的大小 PCL_VISUALIZER_COLOR,                 /**<
  //   3 floats (R, G, B) going from 0.0 (dark) to 1.0 (light) */  设置颜色 PCL_VISUALIZER_REPRESENTATION,
  //   设置渲染的方式 点线面适用于形状的属性
  // };
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
    seg.setAxis(Eigen::Vector3f(0, 1, 0));
    //该角度为平面与轴的多方位夹角，结果为整个所设置的角度中内点最多的
    seg.setEpsAngle(0.7);       //  如果要找出很垂直与y轴的立面角度设置为0.2，最大不要超过0.785
    seg.setMaxIterations(200);  //设置为500次
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
    // cout << "第" << q++ << "次循环拟合立面平云点的个数" << cloud_cluster2->points.size() << endl;
  } while (cloud_cluster2->points.size() > 200);

  if (STEP == 'C')  //去除立面点云
  {
    showdianyungreen(cloud_clusterorgin);
    showdianyunred(cloud_ptr);
  }
}

bool Detectpobeijing(pcl::PointCloud<pcl::PointXYZ>::Ptr origin)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clusterorgin(new pcl::PointCloud<pcl::PointXYZ>);  //保存原始点云
  cloud_clusterorgin = origin;
  // a为x轴滤波参数，b为y轴滤波参数
  pcl::PassThrough<pcl::PointXYZ> pass;  //创建滤波器对象
  pass.setInputCloud(origin);            //设置待滤波的点云
  pass.setFilterFieldName("x");          //设置在x轴方向上进行滤波
  pass.setFilterLimits(1, 5);            //设置滤波范围
  pass.setFilterLimitsNegative(false);   //保留范围
  pass.filter(*origin);                  //滤波并存储
  pass.setInputCloud(origin);            //设置待滤波的点云
  pass.setFilterFieldName("y");          //设置在y轴方向上进行滤波
  pass.setFilterLimits(-4, 4);           //设置滤波范围
  pass.setFilterLimitsNegative(false);   //保留范围
  pass.filter(*origin);
  pass.setInputCloud(origin);           //设置待滤波的点云
  pass.setFilterFieldName("z");         //设置在z轴方向上进行滤波
  pass.setFilterLimits(-1, 3);          //设置滤波范围
  pass.setFilterLimitsNegative(false);  //保留范围
  pass.filter(*origin);                 //滤波并存储
  cout << "直通滤波"
       << " " << origin->points.size() << endl;

  if (STEP == 'B')
  {
    showdianyungreen(origin);  //坡面
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clusterfilter(new pcl::PointCloud<pcl::PointXYZ>);  //保存原始点云
  cloud_clusterfilter = origin;

  //移除origin点云中的平面
  Removeplane(origin);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers(new pcl::PointIndices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1(new pcl::PointCloud<pcl::PointXYZ>);  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster2(new pcl::PointCloud<pcl::PointXYZ>);  
    // pcl::PointCloud<pcl::PointXYZ>::Ptr po1(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr po2(new pcl::PointCloud<pcl::PointXYZ>);
  vector<pcl::PointCloud<pcl::PointXYZ>> v_clouds;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
  cloud_ptr = origin;

  //第一次取0.2,从0.2每次累加0.1
  double angle = 0.2;
  for (int i = 1; i <= 6; i++)
  {
    ne.setSearchMethod(tree1);
    ne.setInputCloud(cloud_ptr);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);  //法向量与轴平行，平面与轴垂直
    seg.setMethodType(6);
    // const static int SAC_RANSAC  = 0;
    // const static int SAC_PROSAC  = 6; 优先采用,迭代速度较快

    //设置轴为z轴
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    //该角度为平面与轴的多方位夹角，结果为整个所设置的角度中内点最多的

    // cout<<"第"<<i<<"次angle的值"<<angle<<endl;
    seg.setEpsAngle(angle);  //  如果要找出很垂直与x轴的立面角度设置为0.2，最大不要超过0.785
    angle += 0.1;
    seg.setMaxIterations(500);       //设置为500次
    seg.setDistanceThreshold(0.01);  //单位米，点到模型的距离
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
    extract1.filter(*cloud_cluster2);  //用于控制循环结束

    if (cloud_cluster2->points.size() > 500)
    {

      pcl::PointCloud<pcl::PointXYZ> cloud0;//点云对象
      cloud0=*cloud_cluster1;
      cloud_ptr = cloud0.makeShared();  //说明拟合出了坡面更新一下cloud_ptr,以备下次使用

      //储存坡面
      pcl::PointCloud<pcl::PointXYZ> cloud;//点云对象
      cloud=*cloud_cluster2;
      v_clouds.push_back(cloud);
      cout << "第" << i << "次"
           << "拟合坡面平云点的个数" << cloud_cluster2->points.size() << endl;
      Eigen::Vector3f v1 = { 0, 0, 1 };
      Eigen::Vector3f v2;
      v2[0] = coefficients->values[0];
      v2[1] = coefficients->values[1];
      v2[2] = coefficients->values[2];
      Eigen::Vector3f v3 = { 0, 1, 0 };
      //默认为false时，结果为弧度
      //为true时，结果为度
      double ang = pcl::getAngle3D(v1, v2, true);
      cout << "拟合的坡面的法向量与水平面的法向量夹角为：" << ang << endl;
    }

    // if (STEP == 'D'&&i==1)
    // {
    //   showdianyungreen(cloud_cluster1);//原始点云
    //   showdianyunred(cloud_cluster2); //坡面
    // }
    // if (STEP == 'E'&&i==2)
    // {
    //   showdianyungreen(cloud_cluster1);//原始点云
    //   showdianyunred(cloud_cluster2); //坡面
    // }
  }
  //  if (STEP == 'E')
  //   {
  //     showdianyunset(v_clouds);
  //   }


  cout << "共拟合的坡面个数" << v_clouds.size() << endl;
  if (STEP == 'D')
  {
    showdianyunred(cloud_clusterfilter); 
    showdianyunset(v_clouds);
  }

    /*
  clock_t begin = clock();
  bool resultflag = false;

  // 1.直通滤波剪切点云
  Directfilter(origin, 2, 10);

  // cloud_clusterfilter直通滤波后的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clusterfilter(new pcl::PointCloud<pcl::PointXYZ>);  //保存原始点云
  cloud_clusterfilter = origin;

  if (origin->points.size() > 1000)
  {
    //移除origin点云中的垂直的栏杆立面
    Removeplane(origin);
  }
  else
  {
    return false;
  }

  // for循环拟合坡面
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers(new pcl::PointIndices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster2(new pcl::PointCloud<pcl::PointXYZ>);
  vector<pcl::PointCloud<pcl::PointXYZ>> v_clouds;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
  cloud_ptr = origin;

  //第一次取0.2,从0.2每次累加0.1
  double angle = 0.2;
  for (int i = 1; i <= 6; i++)
  {
    ne.setSearchMethod(tree1);
    ne.setInputCloud(cloud_ptr);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);  //法向量与轴平行，平面与轴垂直
    seg.setMethodType(6);
    // const static int SAC_RANSAC  = 0;
    // const static int SAC_PROSAC  = 6; 优先采用,迭代速度较快

    //设置轴为z轴
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    //该角度为平面与轴的多方位夹角，结果为整个所设置的角度中内点最多的

    // cout<<"第"<<i<<"次angle的值"<<angle<<endl;
    seg.setEpsAngle(angle);  //  如果要找出很垂直与x轴的立面角度设置为0.2，最大不要超过0.785
    angle += 0.1;
    seg.setMaxIterations(500);       //设置为500次
    seg.setDistanceThreshold(0.01);  //单位米，点到模型的距离
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
    extract1.filter(*cloud_cluster2);  //用于控制循环结束

    // cloud_cluster2为坡面点云同时认为坡面点云大于1000
    if (cloud_cluster2->points.size() > 1000)
    {
      pcl::PointCloud<pcl::PointXYZ> cloud0;  //点云对象
      cloud0 = *cloud_cluster1;
      cloud_ptr = cloud0.makeShared();  //说明拟合出了坡面更新一下cloud_ptr,以备下次使用
      //储存坡面
      pcl::PointCloud<pcl::PointXYZ> cloud;  //点云对象
      cloud = *cloud_cluster2;
      v_clouds.push_back(cloud);
      cout << "第" << i << "次"
           << "拟合坡面平云点的个数" << cloud_cluster2->points.size() << endl;
      Eigen::Vector3f v1 = { 0, 0, 1 };
      Eigen::Vector3f v2;
      v2[0] = coefficients->values[0];
      v2[1] = coefficients->values[1];
      v2[2] = coefficients->values[2];
      //默认为false时，结果为弧度
      //为true时，结果为度
      double ang = pcl::getAngle3D(v1, v2, true);
      cout << "拟合的坡面的法向量与水平面的法向量夹角为：" << ang << endl;

      Eigen::Vector4f centroid;  //质心
      pcl::compute3DCentroid(*cloud_cluster2, centroid);
      //储存本程序坐标
      p1=Eigen::Vector3d(centroid.x(),centroid.y(),centroid.z());
      pcl::PointXYZ min_p1, max_p1;
      pcl::getMinMax3D(*cloud_cluster2, min_p1, max_p1);
      p2=Eigen::Vector3d(min_p1.x,min_p1.y,min_p1.z);
      p3=Eigen::Vector3d(max_p1.x,max_p1.y,max_p1.z);   
    }
  }
  cout << "共拟合的坡面个数" << v_clouds.size() << endl;

  if (STEP == 'D')
  {
    showdianyunred(cloud_clusterfilter);
    showdianyunset(v_clouds);
  }

  return v_clouds.size() > 0 ? true : false;

  */
}

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
    cout << "----------------正在进行一次斜坡检测------------------" << endl;
    flagpoG = Detectpobeijing(preorigin);
    return;
  }
  else
  {
    cout << "原始点云大小小于5000，等待进行下一次检测" << endl;
  }
  return;
}

void DrawCube(Eigen::Vector3d p1,Eigen::Vector3d p2,Eigen::Vector3d p3,ros::Time s)
{
    visualization_msgs::Marker marker;
    //发送质心坐标系到rviz
    // 设置帧 ID和时间戳
    marker.header.frame_id = "map";
    marker.header.stamp = s;
    // 设置该标记的命名空间和ID，ID应该是独一无二的
    // 具有相同命名空间和ID的标记将会覆盖前一个
    marker.ns = "basic_shapes_po";
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
      marker.scale.x = p3(0)-p2(0);
      marker.scale.y = p3(1)-p2(1);
      marker.scale.z = p3(2)-p2(2);

      //设置标记颜色，确保alpha（不透明度）值不为0
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1;
      marker.lifetime = ros::Duration();
      marker_pub.publish(marker);
      cout << "发送第" << i << "个坡marker" << endl;
}

float r_w, r_x, r_y, r_z,p_x, p_y, p_z;
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
  if(!flagpoG)
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
   
  DrawCube(p1q,p2q,p3q,odomMsg->header.stamp);
}

int main(int argc, char **argv)
{
  //初始化viewer1
  viewer1->addCoordinateSystem(0.5);
  // RGB色，0为没有颜色即黑色，全为255则为白色
  viewer1->setBackgroundColor(255, 255, 255);  //设置背景色为白色
  if (argv[1] != NULL)
  {
    STEP = argv[1][0];
  }
  ros::init(argc, argv, "detect_po");
  ros::NodeHandle nh;
  //ros::Subscriber sub = nh.subscribe("/hesai/pandar", 1, chatterCallback);
  ros::Subscriber sub = nh.subscribe("/rslidar_points", 1, chatterCallback);
  //ros::Subscriber sub = nh.subscribe("/partMap", 1, chatterCallback);
  ros::Subscriber subcoordinate = nh.subscribe("lio_sam/mapping/odometry_incremental", 1, odometryHandler);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker_po", 1);
  // waitting
  ros::Rate loop_rate(100);
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
    }
    //休眠
    loop_rate.sleep();
  }
  return 0;
}

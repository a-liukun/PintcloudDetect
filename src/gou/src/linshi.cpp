//头文件：
#include <Eigen/Geometry>
using namespace Eigen;

// topic : / partMap : 以当前雷达位置为中心，大小为10米 * 10米的正方形区域内的 局部地图，其点云坐标值均转到雷达坐标系下 
//         /partMap_odom: 发布 /partMap局部地图时的激光里程计信息 以上信息每5秒传一次
//         注：比如第15秒，当前点云是5～15秒这10秒的数据，全部转到当前（即第15秒）雷达坐标系下，同时也发出当前激光里程计信息

//     /partMap_odom回调函数： 
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
  p_x = odomMsg->pose.pose.position.x;
  p_y = odomMsg->pose.pose.position.y;
  p_z = odomMsg->pose.pose.position.z;
  r_x = odomMsg->pose.pose.orientation.x;
  r_y = odomMsg->pose.pose.orientation.y;
  r_z = odomMsg->pose.pose.orientation.z;
  r_w = odomMsg->pose.pose.orientation.w;
}
// 注：这7个值定义成float型 转换过程：
Quaterniond q2(r_w, r_x, r_y, r_z);
Vector3d t2(p_x, p_y, p_z);
Isometry3d T2w(q2);
T2w.pretranslate(t2);
// Vector3d p1(p.x,p.y,p.z);   //p:全局点  p1:全局坐标值
// Vector3d p2 = T2w.inverse()*p1;  // p2:雷达系下的坐标值 Vector3d p1(p.x,p.y,p.z);

// p:雷达系下的点  p1:雷达系下的坐标值 Vector3d p2 = T2w*p1;
// p2:全局坐标值 注：你是从雷达系转全局系！！！
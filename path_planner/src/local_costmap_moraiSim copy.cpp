#include "ros/ros.h"
#include "ros/package.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "../include/global_path_planner/pathPlanner.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include "std_msgs/Int16.h"

int obsatcle_boundary = 18;
int front_obsatcle_boundary = 30;
#define occupancy_weight 20
#define lidar_gps_offset 1.15 //meter
int max_global_path = 20;     //meter
//string previous =
#define min_global_path 2 //meter
#define PI 3.14159265359
using namespace std;
//using namespace cv;
#define erase_path 30
#define OFFSET_X 302533.174487 // MORAI_utmEastOffset=302459.942        Heading 증가방향  = CCW
#define OFFSET_Y 4124215.34631  // MORAI_utmNorthOffset=4122635.537
#define OFFSET_IRL_TO_MORAI_X 73.232487 //morai 기준과 irl 기준간 차이(irl - morai)
#define OFFSET_IRL_TO_MORAI_Y 1579.80931 //morai 기준과 irl 기준간 차이(irl - morai)
# define M_PI		3.14159265358979323846	/* pi */

bool region_map_loaded = false;
bool avoid_loaded = false;

string trffic_state_string = "go";
const string state_table[6] = {"go", "stop", "cross_road", "wait_for_traffic_light", "static_obs", "outbreak_obs"};
string gps_state = "narrow_int";

nav_msgs::Path previous_path;
ros::Publisher local_path_pub;
ros::Publisher local_costmap;
ros::Publisher local_obstacle_state;
ros::Publisher start_point_pub;
ros::Publisher goal_point_pub;
ros::Publisher stateMarker_pub;
ros::Publisher region_state_pub;
ros::Publisher morai_odom_pub;

int pre_boundary_f = 0;
int over_path_count = 0;
nav_msgs::Path::ConstPtr global_path;
nav_msgs::Path::ConstPtr avoid_path;
nav_msgs::Odometry::ConstPtr currentPose;
nav_msgs::Odometry odomPubData;

nav_msgs::Path globalMapPoses;
pathPlanner kcity_planner;
//pixel
int size_front = lookforward;
int size_side = lookside;
int global_path_index = 0;
const int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
const int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
const int h_func[8] = {14, 10, 14, 10, 10, 14, 10, 14};
bool shortest_path_searched = false;
 Eigen::Quaternionf qinput;
Eigen::Quaternionf toQuaternion(double roll, double pitch, double yaw)
{
  yaw = (yaw+90.0)*M_PI/180.0;

  // Abbreviations for the various angular functions
  // double cy = cos(yaw * 0.5);
  // double sy = sin(yaw * 0.5);
  // double cr = cos(roll * 0.5);
  // double sr = sin(roll * 0.5);
  // double cp = cos(pitch * 0.5);
  // double sp = sin(pitch * 0.5);

  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  // q.w() = cy * cr * cp + sy * sr * sp;
  // q.x() = cy * sr * cp - sy * cr * sp;
  // q.y() = cy * cr * sp + sy * sr * cp;
  // q.z() = sy * cr * cp - cy * sr * sp;
  return q;
}
// gps_utm에서의 pose값 받음
// void poseCallback(const geometry_msgs::PoseWithCovariance::ConstPtr &msg)
// {
//   ROS_INFO("ASDSADSADSADSA");
//   kcity_planner.pose_count++;

//   //utm에서 받아온 pose로 현재pose 업데이트
//   std::queue<geometry_msgs::PoseWithCovariance::ConstPtr> temp_q;
//   temp_q.push(msg);
//   currentPose = temp_q.front();
//   kcity_planner.carpose.covariance = msg->covariance;
//   kcity_planner.carpose.pose = msg->pose;
//   kcity_planner.carpose.pose.position.x -= 302533.174487;
//   kcity_planner.carpose.pose.position.y -= 4124215.34631;

//   //gps상태 check
//   // if (msg->covariance[0] > 1.0 || msg->covariance[7] > 1.0 || msg-> covariance[14] > 2.0)
//   // {
//   //   kcity_planner.gps_stable = "unstable";
//   //     ROS_INFO("GPS unstable");

//   // }
//   kcity_planner.gps_stable = "stable";
//   kcity_planner.crrX = currentPose->pose.position.x - 302533.174487;
//   ;
//   kcity_planner.crrY = currentPose->pose.position.y - 4124215.34631;
//   ;

//   temp_q.pop();
// }

// global path에서 받아온값 global_path변수에 저장
void globalpathCallback(const nav_msgs::Path::ConstPtr &msg)
{
  if (!kcity_planner.globalPath_loaded)
  {
    std::queue<nav_msgs::Path::ConstPtr> temp_q;
    temp_q.push(msg);
    global_path = temp_q.front();
    temp_q.pop();
    int asd = global_path->poses.size();
    ROS_INFO("POSE SIZE : %d", asd);
    for (int idx = 0; idx < global_path->poses.size(); idx++)
    {
      globalMapPoses.poses.push_back(global_path->poses.at(idx));
    }
  }
  kcity_planner.globalPath_loaded = true;
}

//obstacle
void obstacleCallback(const sensor_msgs::PointCloud2ConstPtr &scan)
{
  //sensor_msgs/PointCloud2 data 데이터를 pcl/PointCloud데이터로 변환
  pcl::fromROSMsg(*scan, kcity_planner.obstacle_pcl);
}

//state값 저장
// void stateCallback(const std_msgs::StringConstPtr &msg)
// {
//   state_string = msg->data.c_str();
// }

//trfficstate 저장
void trfficstateCallback(const std_msgs::StringConstPtr &msg)
{

  trffic_state_string = msg->data.c_str();
  kcity_planner.trffic_sign_string = msg->data.c_str();
}
void trafiicCallback(const std_msgs::StringConstPtr &msg)
{
  trffic_state_string = msg->data.c_str();
  kcity_planner.traffic_light_state = msg->data.c_str();
}


void moraiStatusCall(const morai_msgs::ERP42InfoConstPtr &msg){

 qinput = toQuaternion(0, 0, msg->yaw);
  //ROS_INFO("x: %lf",kcity_planner.carpose.pose.position.y+OFFSET_IRL_TO_MORAI_Y);

  kcity_planner.pose_count=1;
  kcity_planner.carpose.pose.position.x = msg->position_x - OFFSET_IRL_TO_MORAI_X;
  kcity_planner.carpose.pose.position.y = msg->position_y - OFFSET_IRL_TO_MORAI_Y;
  kcity_planner.carpose.pose.position.z = 0;
  kcity_planner.carpose.pose.orientation.w = qinput.w();
  kcity_planner.carpose.pose.orientation.x = qinput.x();
  kcity_planner.carpose.pose.orientation.y = qinput.y();
  kcity_planner.carpose.pose.orientation.z = qinput.z();
  kcity_planner.crrX = msg->position_x- OFFSET_IRL_TO_MORAI_X;
  kcity_planner.crrY = msg->position_y- OFFSET_IRL_TO_MORAI_Y;
  
  kcity_planner.gps_stable = "stable";
}
void parkingNumCallback(const std_msgs::Int16ConstPtr &msg)
{
  kcity_planner.parking_number = msg->data;
}
void parking_stateCallback(const std_msgs::Int16 msg)
{
  kcity_planner.parking_state = msg.data;
  if(kcity_planner.parking_state == 4)
  {
      kcity_planner.parking_done = true;
  }
}
int main(int argc, char **argv)
{
  //ros init
  ros::init(argc, argv, "local_costmap");
  ros::NodeHandle n;
  local_path_pub = n.advertise<nav_msgs::Path>("local_path", 1);
  start_point_pub = n.advertise<geometry_msgs::PoseWithCovariance>("start_point", 1);
  local_costmap = n.advertise<nav_msgs::OccupancyGrid>("local_costmap", 1); //차량 기준
  local_obstacle_state = n.advertise<std_msgs::String>("local_obs_state", 1);
  stateMarker_pub = n.advertise<visualization_msgs::Marker>("state_marker", 1);
  region_state_pub = n.advertise<std_msgs::String>("/state", 1);
  morai_odom_pub = n.advertise<nav_msgs::Odometry>("/morai_odom",1);

  //subscriber 정의
  ros::Subscriber global_path_sub = n.subscribe("/global_path", 10, globalpathCallback);
  // ros::Subscriber region_state_sub = n.subscribe("/state", 10, stateCallback);
  ros::Subscriber trffic_region_state_sub = n.subscribe("/traffic_region_state", 10, trfficstateCallback);
  ros::Subscriber trffic_light_sub = n.subscribe("/perception/Detection/TrafficSign", 10, trafiicCallback);
  ros::Subscriber morai_status_sub = n.subscribe("/vehicleStatus",10,moraiStatusCall);
  // ros::Subscriber pose_sub = n.subscribe("/gps_utm_odom", 10, poseCallback);
  // ros::Subscriber pose_sub = n.subscribe("/Perception/Localization/LocalPose", 10, poseCallback);
  // ros::Subscriber obstacle_sub = n.subscribe("/Lidar/obj_pcl", 10, obstacleCallback);
  ros::Subscriber obstacle_sub = n.subscribe("/velo/heightmap", 10, obstacleCallback);
  ros::Subscriber parking_num_sub = n.subscribe("/parking_num",10, parkingNumCallback);
  ros::Subscriber parking_state_sub = n.subscribe("/parking_state", 10, parking_stateCallback);

  //서비스 파라미터 정의
  ros::ServiceClient map_client1 = n.serviceClient<nav_msgs::GetMap>("/global_map/static_map");
  nav_msgs::GetMap srv;
  ros::ServiceClient map_client2 = n.serviceClient<nav_msgs::GetMap>("/plain_map/static_map");
  nav_msgs::GetMap srv_plain;
  ros::ServiceClient map_client3 = n.serviceClient<nav_msgs::GetMap>("/region_map/static_map");
  nav_msgs::GetMap srv_region;
  ros::ServiceClient map_client4 = n.serviceClient<nav_msgs::GetMap>("/obs_map/static_map");
  nav_msgs::GetMap srv_obs;


  string arg_str = "unknown";

  //argc argv란?
  //cout << argc;
  if (argc == 2)
  {
    arg_str = argv[1];
  }

  // 무슨변수?
  int tilting = 28;

  if (arg_str == "final_path")
  {
    tilting = 120;
    //  front_obsatcle_boundary = front_obsatcle_boundary + 10;
    //  obsatcle_boundary = obsatcle_boundary;
  }

  ros::Rate r(10);
  bool state_ok = true;
  ros::Duration duration(1. / 24.);
  while (ros::ok())
  {
	  if (kcity_planner.obsMap_loaded == false)
    {
      if (map_client4.call(srv_obs))
      {
        //cout << "obs_call!" << endl;
        kcity_planner.obs_map = srv_obs.response.map;
        kcity_planner.obs_map.data = srv_obs.response.map.data;
        //cout << obs_map.info.width << " x " << obs_map.info.height << endl;
        kcity_planner.obsMap_loaded = true;
        std::string file_path = ros::package::getPath("global_path_planner");
        kcity_planner.obs_image = cv::imread(file_path + "/map_data/DYNAMIC_OBS_REGION" + ".png", 1);
        //kcity_planner.specific_traffic_obs_image = cv::imread(file_path + "/map_data/only_left.png", 1);
      }
      else
      {
        cout << "failed to load obsMap map!" << endl;
        r.sleep();
        continue;
      }
    }
    if (kcity_planner.regionMap_loaded == false)
    {
      if (map_client3.call(srv_region))
      {
        //cout << "region_call!" << endl;
        kcity_planner.region_map = srv_region.response.map;
        kcity_planner.region_map.data = srv_region.response.map.data;
        //cout << region_map.info.width << " x " << region_map.info.height << endl;
        kcity_planner.regionMap_loaded = true;
        std::string file_path = ros::package::getPath("global_path_planner");
        kcity_planner.region_image = cv::imread(file_path + "/map_data/region_map_20" + ".png", 1);
        //kcity_planner.specific_traffic_region_image = cv::imread(file_path + "/map_data/only_left.png", 1);
      }
      else
      {
        cout << "failed to load region map!" << endl;
        r.sleep();
        continue;
      }
    }
    // global_map을 서비스통신을 통해 한번만 받아오기 위함
    if (kcity_planner.globalMap_loaded == false)
    {
      if (map_client1.call(srv))
      {
        //cout << global_map.info.width << " x " << global_map.info.height << endl;
        kcity_planner.global_map = srv.response.map;
        kcity_planner.global_map.data = srv.response.map.data;
        kcity_planner.globalMap_loaded = true;
      }
      else
      {
        cout << "failed to load global map!" << endl;
        r.sleep();
        continue;
      }
    }

    // //plain_map을 서비스통신을 통해 한번만 받아오기 위함
    // if (kcity_planner.plainMap_loaded == false)
    // {
    //   if (map_client2.call(srv_plain))
    //   {
    //     //cout << "plain_call!" << endl;
    //     kcity_planner.plain_map = srv_plain.response.map;
    //     kcity_planner.plain_map.data = srv_plain.response.map.data;
    //     //cout << plain_map.info.width << " x " << plain_map.info.height << endl;
    //     kcity_planner.plainMap_loaded = true;
    //   }
    //   else
    //   {
    //     //cout << "failed to load plain map!" << endl;
    //     r.sleep();
    //     continue;
    //   }
    // }
    
    //region_map을 서비스 통신을 통해 한번만 받아오기 위함
   if(kcity_planner.pose_count!=0){
    Eigen::Matrix4d gps = Eigen::Matrix4d::Identity(); //gps to vehicle centor

    gps.block<3, 3>(0, 0) = Eigen::Quaterniond(qinput.w(), qinput.x(), qinput.y(), qinput.z()).toRotationMatrix();
    gps.block<3, 1>(0, 3) = Eigen::Vector3d(kcity_planner.crrX, kcity_planner.crrY, 0);

    //gps.block<3, 1>(0, 3) = Eigen::Vector3d(270, -445, 0);


    //Eigen::Vector3d GPS_to_vehicle(1.1,-0.44,0);
    Eigen::Vector3d GPS_to_vehicle(0,0,0);
    Eigen::Vector3d GPS(kcity_planner.crrX,kcity_planner.crrY,0);
    //Eigen::Vector3d GPS_to_vehicle(utm_x,utm_y,0);
    //Eigen::Vector3d GPS(-494,-200,0);

    Eigen::Vector3d RP = gps.block<3, 3>(0, 0) * GPS_to_vehicle;
    Eigen::Vector3d vehicle_ = RP + GPS; // world position

    odomPubData.header.frame_id = "map";
    odomPubData.child_frame_id = "novatel";
    odomPubData.header.stamp = ros::Time::now();
    odomPubData.pose.pose.position.x = vehicle_(0);
    odomPubData.pose.pose.position.y = vehicle_(1);
    odomPubData.pose.pose.orientation.x = qinput.x();
    odomPubData.pose.pose.orientation.y = qinput.y();
    odomPubData.pose.pose.orientation.z = qinput.z();
    odomPubData.pose.pose.orientation.w = qinput.w(); 
    for (size_t i = 0; i < 35; i++)
    {
        odomPubData.pose.covariance.at(i) =0; // initialize cov matrix
    }
    morai_odom_pub.publish(odomPubData);

    }
    switch (kcity_planner.currentstate)
    {
    case GLOBAL_PATH_TRACKING:

      //draw pcl utm
      //lane watch dog
      //
      if (kcity_planner.globalPath_loaded)
      kcity_planner.globalPathTracking(globalMapPoses, local_path_pub); // gp추종
      kcity_planner.checkState(region_state_pub);                                         //현재 state 체크하기
      kcity_planner.drawStateMarker(stateMarker_pub);
      //check state
      break;

    case STOP_READY:
      //draw pcl utm
      //check traffic sign & sign_flag
      //check obstacle & obs_flag
      //if flag == 1 change state to STOP
      kcity_planner.globalPathTracking(globalMapPoses, local_path_pub);
      kcity_planner.checkState(region_state_pub); //현재 state 체크하기
      kcity_planner.drawStateMarker(stateMarker_pub);
      break;
 
    case STOP:
      //draw pcl utm
      //(check sign *OR* Obstacle) and change flag
      kcity_planner.checkState(region_state_pub); //현재 state 체크하기
      kcity_planner.drawStateMarker(stateMarker_pub);
      //
      break;
    // case OBSTACLE_AVOIDANCE_READY:
    //   //draw pcl utm
    //   //check obstacle & flag
    //   //if flag == 1 change state to Obs_avoidance
    //   break;

    // case OBSTACLE_AVOIDANCE:
    //   //make local grid map
    //   //draw new path
    //   //draw pcl utm
    //   //publish path
    //   //do it while region !=obs_avoidance
    //   break;

    case PARKING_READY:
      kcity_planner.globalPathTracking(globalMapPoses, local_path_pub);
      kcity_planner.checkState(region_state_pub); //현재 state 체크하기
      kcity_planner.drawStateMarker(stateMarker_pub);
      //decrease velocity
      //find empty parking space
      //if find space change state to PARKING
      break;

    case PARKING:
      kcity_planner.checkState(region_state_pub); //현재 state 체크하
      kcity_planner.drawStateMarker(stateMarker_pub);
      //follow parking path (while get checkpoint)
      //delay 10s
      //follow paring path (while get checkpoint)
      //parkingEndFlag = 1, chage state to GLOBAL_PATH_TRACKING
      break;
    case OBS_READY:
      kcity_planner.outbreakStopCheck();
      kcity_planner.checkState(region_state_pub); //현재 state 체크하
      kcity_planner.drawStateMarker(stateMarker_pub);
      kcity_planner.globalPathTracking(globalMapPoses, local_path_pub);

    // case UNSTABLE_GPS:
    //   // check gps accurancy
    //   //
    default:
      kcity_planner.drawStateMarker(stateMarker_pub);
      kcity_planner.checkState(region_state_pub); //현재 state 체크하기
      kcity_planner.globalPathTracking(globalMapPoses, local_path_pub);
      break;
    }
  
    // local_path_pub.publish(temp_path);
    // temp_local_map.data = temp_data;
    // local_costmap.publish(temp_local_map);

    r.sleep();
    ros::spinOnce();
  }
  ros::spin();
  return 0;
}

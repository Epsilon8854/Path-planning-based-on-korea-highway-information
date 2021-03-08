#ifndef _LOCALCOSTMAP
#define _LOCALCOSTMAP


//made by 이인하 모르겠으면 단톡방에 물어봐줘요

#include "ros/ros.h"
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
#include <tf/transform_broadcaster.h>
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
#include <morai_msgs/ERP42Info.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include "std_msgs/Int16.h"
#include <tf/transform_broadcaster.h>

//! @brief state 정의, path planner 피피티 참조!
// enum state
// {
//   GLOBAL_PATH_TRACKING, //255 0
//   STOP,                 //33  1
//   STOP_READY,           //33  2
//   OBSTACLE_AVOIDANCE,   //49  3
//   OBSTACLE_AVOIDANCE_READY,//49
//   PARKING,//85
//   PARKING_READY,//85
//   UNSTABLE_GPS//200
// };

enum state //0~7번은 위치(맵) 기반으로 나눈 state들, 나머지는 차량의 동작, 상태에 따른 state
{
  OBS_READY,                //21 -> 11 0 노란색 | 동적 장애물
  STOP_READY,               //33 -> 31  1 주황색 | 신호등
  OBSTACLE_AVOIDANCE_READY, //49 -> 51  2 하늘색 | 정적 장애물
  PARKING_READY,            //85 -> 71  3 연보라 | 주차
  CROSSWORK,                //102 -> 91  4 핑크색 | 횡단보도
  END,                      //178 -> 111 5 금색 | 끝
  SLOW_DOWN,                //65 ->131  6 하얀색 | GPS 튀는 부분
  NO_LANE,                  //none -> 151 7 민트색 | 차선 인식 불가 영역, 교차로
  LOCAL_CALIBARATION,       //171
  TRAFFIC_LEFT,             //191
  SHORT_OBSTACLE_READY,
  UNSTABLE_GPS,             // 8 회색 | gps 시망...
  GLOBAL_PATH_TRACKING,     // 9 녹색 | gp 만 따라가기
  STOP,                     // 10 빨간색 | 멈추기
  OBSTACLE_AVOIDANCE,       // 11 파란색 | 장애물 회피하기
  LOCALIZATION, //12 밝은 회색 | 위치 calibration 하기
  PARKING // 13 무슨 색 하지.. | 주차하기
};

#define MAX_GLOBAL_PATH_LENGTH 20 //뿌려주는 local path길이, 약 20m, path간 간격이 일정하지 않아서 정확하지 않음
#define lidar_gps_offset2020 1.0
#define bigger_frame 6
#define lookforward 200
#define lookside 60
#define lane_boundary 2 //pixel, 1 pixel = 0.1m^2
#define bus_lane_boundary 8 //pixel, 1 pixel = 0.1m^2

#define obs_boundary 13
#define M_RAD      57.29577951
#define PATHGAP 5.0

class pathPlanner
{
private:
  // 현재 자신의 state
  int gp_start_index,gp_end_index;
  std::string state_string;
  int table_index;
  std_msgs::String msg;

  //

public:
  ros::Time plannerTime;
  state currentstate; //현재 차량의 state
  std::string gps_stable;
  std::string trffic_sign_string;
  double crrX, crrY; // 현재 차량의 위치 (utm 기준)
  int pose_count;
  int id;
  geometry_msgs::PoseWithCovariance carpose;                //pose는 operater가 따로 없어서... covariance 정보만 있는변수
  bool globalPath_loaded;                                   // 주행 코스 로드 확인용
  bool globalMap_loaded, plainMap_loaded, regionMap_loaded,obsMap_loaded,signalStopMap_loaded; // grid 맵 로드 확인용
  nav_msgs::OccupancyGrid global_map;                    // 글로벌 맵
  nav_msgs::OccupancyGrid plain_map;                     // 정적 장애물 구간용 맵
  nav_msgs::OccupancyGrid region_map;                    // state 구별용 맵
  nav_msgs::OccupancyGrid obs_map;                       // 동적 장애물 구간용 맵
  nav_msgs::OccupancyGrid signalStop_map;                       // 신호등 정지용 맵

  
  cv::Mat region_image,obs_image,signalStop_image;
  cv::Mat specific_traffic_region_image;
  std::string traffic_light_state;
  int parking_number,parking_state;
  pcl::PointCloud<pcl::PointXYZI> obstacle_pcl;             //pcl 포인트들
  bool parking_done, outBreakObsExist, signalStopFlag,trafficDoneFlag;
  int trafficCnt;
  int state_point;
  double roll, pitch, yaw;
  double current_th,cos_th,sin_th,sin_th_inv,cos_th_inv,path_cos_th,path_sin_th;
  int obs_map_x;
  int obs_map_y;

  int outBreakObsThreshold,outBreakObsCnt,outBreakObsMissingCnt;
  double resolution;
  //정적장애물
  nav_msgs::OccupancyGrid local_map;
  std::vector<double> map_size;
  int xAxis,yAxis; //맵 크기
  double tolerance;
  double gp_yaw; //global path의 기울기
  double local_map_yaw,local_map_pitch,local_map_roll;
  int avoidPathIdx;
  cv::Mat local_grid_image;

  //신호등s
  int signalStopState;
 

  pathPlanner()
  {
    currentstate = GLOBAL_PATH_TRACKING;
    state_string = "go";
    globalPath_loaded = globalMap_loaded = plainMap_loaded = regionMap_loaded = obsMap_loaded = false;
    crrX = crrY = 0;
    pose_count = 0;
    gps_stable = "stable";
    id = 0;
    gp_start_index = -1;
    gp_end_index =0;
    parking_number = 0;
    parking_state =1;
    parking_done = false;
    state_point = 255;
    traffic_light_state = "none";
    roll = pitch = yaw = 0.0;
    current_th = 0.0;
    resolution = 0.1;
    obs_map_x =1824;
    obs_map_y =6260;
    outBreakObsThreshold = 7;
    outBreakObsCnt =0;
    outBreakObsMissingCnt=0;
    outBreakObsExist =false;
    tolerance = 5.0;
    gp_yaw=0;
    avoidPathIdx=12;
    signalStopFlag=false;
    trafficDoneFlag = false;
    trafficCnt =0;
 
  }

  /*! @brief 자기 위치에서 가장 가까운 global path를 publish해주는 함수
    * @param[in] global_path_poses globalpathCallback 받은 전체 global path
    * @param[in] local_path_pub path 보낼 publisher, topic: /local_path
    */
  void globalPathTracking(nav_msgs::Path &global_path_poses, ros::Publisher &local_path_pub);

  /*! @brief 현재 자신의 state를 마커(색깔로 구별)로 맵에 표시하는 함수
    */
  void drawStateMarker(ros::Publisher &stateMarker_pub);

  /*! @brief 현재 state 판단
    */
  void checkState(ros::Publisher &region_state_pub);
  /*! @brief 현재 state 판단
    */
  void makeTF();

  /*! @brief 현재 state 판단
    */
  void checkSignalStop();

  /*---------동적 장애물 미션---------*/

  /*! @brief 동적 장애물 있는지 체크
    */
  void outbreakStopCheck();
  /*-------------------------------*/

  
  
  /*---------정적 장애물 미션---------*/
  
  /*! @brief 정적 장애물 회피 미션 생성
    */
  void obstacleAvoidance(nav_msgs::Path &global_path_poses,ros::Publisher &local_costmap_pub,ros::Publisher &local_path_pub);

  /*! @brief 로컬 코스트 맵 생성
    */
  void makeLocalCostMap(ros::Publisher &local_costmap_pub,nav_msgs::Path &global_path_poses);
  
   /*! @brief 정적 장애물 회피 경로 생성
    */
  void makeLongAvoidancePath(nav_msgs::Path &global_path_poses,ros::Publisher &local_path_pub);
  
  void makeShortAvoidancePath(nav_msgs::Path &global_path_poses,ros::Publisher &local_path_pub);


  /*-------------------------------*/






};
#endif

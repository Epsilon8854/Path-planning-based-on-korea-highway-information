#include "global_path_planner/pathPlanner.h"

void pathPlanner::globalPathTracking(nav_msgs::Path &global_path_poses, ros::Publisher &local_path_pub)
{
  
  //유클리드 거리(내 위치<.global path)용 변수
  double euclidean_distance;
  //유클리드 거리중 가장 짧은 거리
  double shortest_distance = INT32_MAX;
  // int asd = global_path_poses.poses.size();
  // ROS_INFO("POSE SIZE : %d",asd);
  //현재 위치에서 제일 가까운 global path 점을 찾고 그 점을 시작점으로 정함
  //  ROS_INFO("Nearest IDX : %d \n", gp_start_index);
  if (gp_start_index == -1 || gp_start_index == 838) //아직 시작점을 찾지 않은 상태라면
  {
    for (int i = 0; i < global_path_poses.poses.size() - 1; i++) //
    {
      euclidean_distance = sqrt(pow((crrX - global_path_poses.poses.at(i).pose.position.x), 2) + pow((crrY - global_path_poses.poses.at(i).pose.position.y), 2));
      if (euclidean_distance < shortest_distance)
      {
        shortest_distance = euclidean_distance;
        gp_start_index = i;
      }
    }
  }
  else // 이미 시작점을 찾은 경우엔
  {
    //현재 시작점 -5 ~ 20 범위 내에서 새로운 시작점을 찾아냄, gps터진 경우엔 시작점 -1로 초기화
    for (int i = gp_start_index - 20; i < gp_start_index + 20 && i < global_path_poses.poses.size() - 21; i++)
    {
      if (i < 0)
        i = 0;
      euclidean_distance = sqrt(pow((crrX - global_path_poses.poses.at(i).pose.position.x), 2) + pow((crrY - global_path_poses.poses.at(i).pose.position.y), 2));
      if (euclidean_distance < shortest_distance)
      {
        shortest_distance = euclidean_distance;
        gp_start_index = i;
      }
    }
    if(shortest_distance > 30){ //만약 차가 순간이동한 경우(시뮬)
      gp_start_index =-1; //시작점 초기화
    }
  }

  nav_msgs::Path globalPathToFollow; //전체 global path 에서 당장 따라갈 경로만 잘라낸 경로
  globalPathToFollow.header.stamp = ros::Time::now();
  globalPathToFollow.header.frame_id = "/map";

  double far_euclidean = 0.0; //제일 먼 유클리드 거리
  if(gp_start_index!=-1){
  for (int gp_index = gp_start_index; far_euclidean < MAX_GLOBAL_PATH_LENGTH && gp_index < global_path_poses.poses.size() - 21; gp_index++)
  {
    globalPathToFollow.poses.push_back(global_path_poses.poses.at(gp_index));
    far_euclidean = sqrt(pow((crrX - global_path_poses.poses.at(gp_index).pose.position.x), 2) + pow((crrY - global_path_poses.poses.at(gp_index).pose.position.y), 2));
  }
  }
  gp_end_index = gp_start_index + globalPathToFollow.poses.size() -1;
  /*현재 위치에서 가장 가까운 global path를 시작점으로 해서
  길이 20m 만큼 잘라냄 */
  // while (far_euclidean < MAX_GLOBAL_PATH_LENGTH && gp_index < global_path_poses.poses.size())
  // {
  //   globalPathToFollow.poses.push_back(global_path_poses.poses.at(gp_index));
  //   far_euclidean = sqrt(pow((crrX - global_path_poses.poses.at(gp_index).pose.position.x), 2) + pow((crrY - global_path_poses.poses.at(gp_index).pose.position.y), 2));
  //   gp_index++;
  // }
  if(currentstate != OBSTACLE_AVOIDANCE_READY && currentstate != SHORT_OBSTACLE_READY)
    local_path_pub.publish(globalPathToFollow);
  // ROS_INFO("x : %lf | y : %lf \n", crrX, crrY);
   ROS_INFO("Closest : %d | Far : %lf | Short : %lf | mid :%d", gp_start_index, far_euclidean,shortest_distance,gp_end_index);
}

void pathPlanner::drawStateMarker(ros::Publisher &stateMarker_pub)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "car_state";
  marker.id = this->id++;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = carpose.pose;
  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 0.6;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  switch (currentstate) //마커 색칠하기
  {
  case OBS_READY:
    // 노란색
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    break;
  case STOP_READY:
    // 주황색
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    break;
  case OBSTACLE_AVOIDANCE_READY:
    // 하늘색
    marker.color.r = 0.0;
    marker.color.g = 0.8;
    marker.color.b = 1.0;
    break;
  case PARKING_READY:
    // 연보라
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    break;
  case CROSSWORK:
    // 핑크색
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    break;
  case END:
    // 금색
    marker.color.r = 1.0;
    marker.color.g = 0.843;
    marker.color.b = 0.0;
    break;
  case SLOW_DOWN:
    // 하얀색
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    break;
  case NO_LANE:
    // 민트색
    marker.color.r = 0.5;
    marker.color.g = 1.0;
    marker.color.b = 0.83;
    break;
  case UNSTABLE_GPS:
    // 회색
    marker.color.r = 0.3;
    marker.color.g = 0.3;
    marker.color.b = 0.3;
    break;
  case GLOBAL_PATH_TRACKING:
    // 녹색
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    break;
  case STOP:
    //빨간색
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    break;
  case OBSTACLE_AVOIDANCE:
    //파란색
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    break;
  case LOCALIZATION:
    // 밝은 회색
    marker.color.r = 0.7;
    marker.color.g = 0.7;
    marker.color.b = 0.7;
    break;
  }

  //only if using a MESH_RESOURCE marker type:
  stateMarker_pub.publish(marker);
  return;
}
void pathPlanner::checkState(ros::Publisher &region_state_pub)
{
  
  const std::string state_table[11] = {"go", "stop", "cross_road", "slow_down_for_traffic_light", "static_obs", "outbreak_obs", "parking_search", "parking", "xxx", "xxx", "finish"};
  // ROS_INFO("pose count %d",pose_count);

  if (pose_count != 0 && regionMap_loaded)
  {
    double resolution = region_map.info.resolution;

    int region_map_x = (crrX - region_map.info.origin.position.x) / resolution;
    int region_map_y = (crrY - region_map.info.origin.position.y) / resolution;
    cv::Vec3b original_map_data;
    //cv::Vec3b traff_map_data;

    if (region_map_x < region_image.cols && region_image.rows - region_map_y < region_image.rows && region_map_x > 0 && region_image.rows - region_map_y > 0)
    {

      original_map_data = region_image.at<cv::Vec3b>(region_image.rows - region_map_y, region_map_x);
      //cv::imshow("left only",region_image);

      //                cv::waitKey(5);
      state_point = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;
      //int trf_point = (traff_map_data[0] + traff_map_data[1] + traff_map_data[2]) / 3;
                // cout << point << "\n";
      if (state_point > 240)
      {
        table_index = 8;
        currentstate = GLOBAL_PATH_TRACKING;
        state_string = "go";
        table_index = (state_point) / 20;
        // ROS_INFO("1.IDX: %d | point: %d ", table_index, state_point);
        // std::cout << "current_state222 : " << state_string<<currentstate << "\n";
        traffic_light_state = "none";
        trafficDoneFlag = false;
        trafficCnt =0;
      }
      else
      {

        table_index = (state_point) / 20;
          ROS_INFO("2.IDX: %d | point: %d ", table_index, state_point);

        if (table_index >= 0 && table_index < 12)
        {
          currentstate = state(table_index);
          // std::cout << "currentstate : " << currentstate << "\n";

        }        // cout << "obs_state : " << obs_state << "\n";
        // std::cout << "traffic light state : " << traffic_light_state << "\n";
        // cout << "only left? : " << trf_point << "\n";
        if(currentstate !=TRAFFIC_LEFT && currentstate !=STOP_READY){
          traffic_light_state = "none";
          trafficDoneFlag = false;
          trafficCnt =0;
        }
        if (currentstate == STOP_READY && traffic_light_state == "none")
        {
          state_string = "slow_down_for_traffic_light";
        }
        else if (currentstate == STOP_READY && (traffic_light_state == "RED") && trafficDoneFlag == false)
        {
          checkSignalStop();
          if(signalStopFlag == true){
            currentstate = STOP;
            state_string = "stop";
            ROS_INFO("STOP! RED Light");
          }
          else{
            
            state_string = "slow_down_for_traffic_light";
            ROS_INFO("will be stop");
          }
        }
        else if (currentstate == STOP_READY && (traffic_light_state == "GREEN" || traffic_light_state=="LEFT_GREEN"||traffic_light_state=="LEFT_RED"))
        {
          trafficCnt++;
          ROS_INFO("Steady GREEN Light %d",trafficCnt);
          if(trafficCnt>10){
          currentstate = GLOBAL_PATH_TRACKING;
          ROS_INFO("GO! GREEN Light");
          state_string = "go";
          trafficDoneFlag= true;
          }
          else{
            state_string = "slow_down_for_traffic_light";
          }

        }

        if (currentstate == TRAFFIC_LEFT && traffic_light_state == "none")
        {
          state_string = "slow_down_for_traffic_light";
        }
        else if(currentstate == TRAFFIC_LEFT && (traffic_light_state == "RED" || traffic_light_state == "GREEN")&& trafficDoneFlag == false){
          checkSignalStop();
          if(signalStopFlag ==true){
            currentstate = STOP;
            state_string = "stop";
            ROS_INFO("STOP! RED Light");
          }
          else{
            state_string = "slow_down_for_traffic_light";
            ROS_INFO("will be stop");

          }
        }
        else if (currentstate == TRAFFIC_LEFT && (traffic_light_state=="LEFT_GREEN"||traffic_light_state=="LEFT_RED"))
        {
           trafficCnt++;
          ROS_INFO("Steady GREEN Light %d",trafficCnt);
          if(trafficCnt>10){
            currentstate = GLOBAL_PATH_TRACKING;
            ROS_INFO("GO! GREEN Light");
            state_string = "go";
            trafficDoneFlag= true;
          }
          else{
            state_string = "slow_down_for_traffic_light";
          }

        }

       
        //           // ROS_INFO("111");
        if(currentstate == PARKING_READY && !parking_done && parking_number > 0)
        {
          state_string = "parking";
        }
        else  if(currentstate == PARKING_READY && parking_done)
        {
          state_string = "go";
        }
        else if(currentstate == PARKING_READY){
          state_string = "parking_search";
        }

        if(currentstate == OBS_READY && outBreakObsExist == true){
          state_string = "stop";
        }
        else if(currentstate == OBS_READY && outBreakObsExist == false){
          state_string = "outbreak_obs";
        }

        if(currentstate == OBSTACLE_AVOIDANCE_READY||SHORT_OBSTACLE_READY){
          state_string = "static_obs";
        }

        if(currentstate == END){
          state_string = "finish";
        }
        if(currentstate == SLOW_DOWN){
          state_string = "go";
        }
        if(currentstate == NO_LANE){
          state_string = "go";
        }
        if(currentstate == LOCAL_CALIBARATION){
          state_string = "go";
        }

        
      
        // else{
        //   std::cout << "current_state : " << state_string<<currentstate << "\n";
        // }
                          // ROS_INFO("222");

        // state_string = "sibal";

        // if(currentstate == "static_obs")
        // {
        //     if(obs_state == "stuck")
        //     {
        //         currentstate = "static_obs";
        //     }
        //     else {
        //         currentstate = "static_obs";//slow down for obs
        //     }
        // }
        // else if(currentstate == "outbreak_obs")
        // {
        //     if(obs_state == "stuck")
        //     {
        //         currentstate = STOP;
        //     }
        //     else {
        //         currentstate = "outbreak_obs";//slow down for obs
        //     }
        // }
      }
      // std::cout << "current_state : " << state_string<<currentstate << "\n";
      if (gps_stable == "unstable")
      {
        currentstate = UNSTABLE_GPS;
      }
    }

    else
    {
      ROS_INFO("-----------------------------------------------------------");
    }
    // ROS_INFO("-----------------------------------------------------------");
    std::stringstream ss;
    ss << state_string;
    msg.data = ss.str();
    region_state_pub.publish(msg);
  
    return;
  }
}
void pathPlanner::makeTF()
{
  if(gp_start_index == -1)
    return;
  if(pose_count!=0){
    tf::Quaternion q(
                  carpose.pose.orientation.x,
                  carpose.pose.orientation.y,
                  carpose.pose.orientation.z,
                  carpose.pose.orientation.w);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(crrX,crrY,0));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, this->plannerTime, "map", "novatel"));
  }
}
void pathPlanner::checkSignalStop()
{
    int StopSignal_map_x = (crrX - signalStop_map.info.origin.position.x) / resolution;
    int StopSignal_map_y = (crrY - signalStop_map.info.origin.position.y) / resolution;
    cv::Vec3b original_map_data;
    //cv::Vec3b traff_map_data;

    if (StopSignal_map_x < signalStop_image.cols && signalStop_image.rows - StopSignal_map_y < signalStop_image.rows && StopSignal_map_x > 0 && signalStop_image.rows - StopSignal_map_y > 0)
    {

      original_map_data = signalStop_image.at<cv::Vec3b>(signalStop_image.rows - StopSignal_map_y, StopSignal_map_x);

      //                cv::waitKey(5);
      signalStopState = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;
      if(signalStopState <200){
        signalStopFlag = true;
      }
      else if(signalStopState>=200){
        signalStopFlag = false;
      }
    }

}
void  pathPlanner::outbreakStopCheck()
{
  tf::Quaternion q(
                  carpose.pose.orientation.x,
                  carpose.pose.orientation.y,
                  carpose.pose.orientation.z,
                  carpose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  current_th = yaw;
  cos_th = cos(yaw);
  sin_th = sin(yaw);
  obs_map_x = (crrX + 182.4)/resolution;
  obs_map_y = (crrY + 626)/resolution;

  cv::Vec3b original_map_data;

 if(obs_map_x < obs_image.cols && obs_image.rows - obs_map_y < obs_image.rows && obs_map_x>0 && obs_image.rows - obs_map_y>0)
  {
    outBreakObsCnt=0;

      for(int i = 0;i<obstacle_pcl.size();i++)
      {
          double pcl_x = -(obstacle_pcl.at(i).x - lidar_gps_offset2020);
          double pcl_y = -obstacle_pcl.at(i).y;
          if(!(pcl_x >-2.0 && pcl_x<2.0 &&pcl_y > -0.5 &&pcl_y<0.5))
          {

              double rot_x = cos_th*(pcl_x) - sin_th*(pcl_y);
              double rot_y = sin_th*(pcl_x) + cos_th*(pcl_y);
              double global_x = -(rot_x-crrX);
              double global_y = -(rot_y-crrY);
              int map_x = (global_x + 182.4)/resolution; //182랑 626은 맵 정 중앙점 위치, 바로 아래에서
              int map_y = (global_y + 626)/resolution;   // 
              cv::Vec3b original_map_data;
              cv::Vec3b original_pcl_data;
              if(map_x < obs_image.cols && obs_image.rows - map_y < obs_image.rows && map_x>0 && obs_image.rows - map_y>0)
              {
                original_map_data = obs_image.at<cv::Vec3b>(obs_image.rows - map_y, map_x);

              }

              int point = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;
              if(point <230){
                outBreakObsCnt++;
              }
          }
      }
  ROS_INFO("obs points: %d", outBreakObsCnt);
  if(outBreakObsCnt > outBreakObsThreshold){
    ROS_INFO("outbreak obs, STOP");
    outBreakObsExist = true;
    outBreakObsMissingCnt=0;
  }
  else{
    outBreakObsMissingCnt++;
      if(outBreakObsMissingCnt<outBreakObsThreshold){
      ROS_INFO("NO outbreak obs, GO");
      outBreakObsExist = false;
    }
  }

  }
  else {
        ROS_INFO("out of obs map");

  }
}




void pathPlanner::obstacleAvoidance(nav_msgs::Path &global_path_poses,ros::Publisher &local_costmap_pub,ros::Publisher &local_path_pub2)
{
  makeLocalCostMap(local_costmap_pub,global_path_poses);
  if(currentstate == OBSTACLE_AVOIDANCE_READY){
    makeLongAvoidancePath(global_path_poses,local_path_pub2);
  }
  else if(currentstate == SHORT_OBSTACLE_READY){
    makeShortAvoidancePath(global_path_poses,local_path_pub2);
  }
  

}

void pathPlanner::makeLongAvoidancePath(nav_msgs::Path &global_path_poses,ros::Publisher &local_path_pub2)
{
  if(gp_start_index == -1)
    return;
  int zsei = global_path_poses.poses.size();

  // ROS_INFO("Closest : %d | mid :%d | size %d", gp_start_index,gp_end_index,zsei);

  double gpX1 =  global_path_poses.poses.at(gp_start_index).pose.position.x; 
  double gpX2 =  global_path_poses.poses.at(gp_end_index).pose.position.x; 
  double gpY1 =  global_path_poses.poses.at(gp_start_index).pose.position.y; 
  double gpY2 =  global_path_poses.poses.at(gp_end_index).pose.position.y; 

  nav_msgs::Path localPathToFollow;
  localPathToFollow.header.frame_id = "/map";
  localPathToFollow.header.stamp = plannerTime;
  gp_yaw =  atan2(gpY2-gpY1,gpX2-gpX1);
  ROS_INFO("| %f: %f |",gp_yaw, gp_yaw*M_RAD);

  path_cos_th = cos(180/M_RAD - gp_yaw);
  path_sin_th = sin(180/M_RAD - gp_yaw);
  cos_th_inv = cos(-(yaw));
  sin_th_inv = sin(-(yaw));
  int crash_weight[26]={0}; //(-1.5m, -1.0m, -0.5m, 0m, 0.5m ...)
  double rot_x, rot_y;
  int map_index, local_index;
  double path_x, path_y,rot_path_x,rot_path_y,local_map_x,local_map_y;
  
  for(int path_idx = 1; path_idx <15;path_idx++){
      // 충돌판정 및 선택
       for(int lpIdx = gp_start_index; lpIdx <= gp_end_index; lpIdx++){

        // localPathToFollow.poses.push_back(global_path_poses.poses.at(lpIdx));
  
        path_x = (global_path_poses.poses.at(lpIdx).pose.position.x)+path_sin_th*PATHGAP*(path_idx-3)*resolution; //회전 전 local맵에서의 좌표
        path_y = (global_path_poses.poses.at(lpIdx).pose.position.y)+path_cos_th*PATHGAP*(path_idx-3)*resolution;
       
        path_x = path_x - crrX;
        path_y = path_y - crrY;


        rot_path_x = cos_th_inv * path_x  - sin_th_inv * path_y;
        rot_path_y = sin_th_inv * path_x  + cos_th_inv * path_y;

        local_map_y = (double)((double)yAxis - rot_path_x)/resolution; 
        local_map_x = ((double)xAxis/2.0 - rot_path_y)/resolution;
        int tempIMGdata = local_grid_image.at<unsigned char>((int)local_map_y, (int)local_map_x);

        if(local_map_x > 3.0 && local_map_x<177.0 && local_map_y> 20.0 && local_map_y<120.0){
            int adsad = local_grid_image.at<unsigned char>((int)local_map_y, (int)local_map_x);
            // ROS_INFO("path: %d, data : %d ,X: %lf Y:%lf",path_idx,adsad,local_map_x,local_map_y);
          if(tempIMGdata== 255){
            crash_weight[path_idx-1]+= 100 + (path_idx - avoidPathIdx)*(path_idx - avoidPathIdx);
            crash_weight[path_idx]+= 150 + (path_idx - avoidPathIdx)*(path_idx - avoidPathIdx);
            crash_weight[path_idx+1]+= 100 + (path_idx - avoidPathIdx)*(path_idx - avoidPathIdx);
          }
        }
        else if(local_map_x > 3.0 && local_map_x<177.0 && local_map_y> 120.0 && local_map_y<147.0){
          int adsad = local_grid_image.at<unsigned char>((int)local_map_y, (int)local_map_x);
          ROS_INFO("path: %d, data : %d ,X: %lf Y:%lf",path_idx,adsad,local_map_x,local_map_y);

          
        
          if(tempIMGdata== 255){
            if(path_idx<=avoidPathIdx){ //장애물이 왼쪽에 있으면
              for(int weightIdx = 0; weightIdx <=path_idx; weightIdx++){
                crash_weight[weightIdx]+= 500 + (path_idx - avoidPathIdx)*(path_idx - avoidPathIdx);
              }
            }
            else if(path_idx>=avoidPathIdx){ //장애물이 오른쪽에 있으면
              for(int weightIdx =path_idx; weightIdx <15; weightIdx++){
                crash_weight[weightIdx]+= 500 + (path_idx - avoidPathIdx)*(path_idx - avoidPathIdx);
              }
            }
          }
        }
      }
  }

  for(int path_idx = 0; path_idx <15;path_idx++){
        crash_weight[path_idx] += abs(path_idx - avoidPathIdx);
    ROS_INFO("| %d: %d |",path_idx, crash_weight[path_idx]);
  }

  int MinObsSum = 999999;//구구구 비둘기야 밥ㅂ머
  for(int path_idx = 1; path_idx <14;path_idx++){
    int avg = (crash_weight[path_idx-1] + crash_weight[path_idx] + crash_weight[path_idx+1])/3;
    if(MinObsSum > avg){
      avoidPathIdx = path_idx;
      MinObsSum = avg;
    }
  }
  
  if(crash_weight[2] < 15&&crash_weight[3] < 15&&crash_weight[4] < 15){
    avoidPathIdx = 3;
  }

    ROS_INFO("-------------LongPATH : %d-------------- ",avoidPathIdx);

  // for(int lpIdx = gp_start_index; lpIdx <= gp_end_index; lpIdx++){

  //         localPathToFollow.poses.push_back(global_path_poses.poses.at(lpIdx));
  //         localPathToFollow.poses.at(lpIdx-gp_start_index).pose.position.x +=path_sin_th*PATHGAP*(avoidPathIdx-5)*resolution;
  //         localPathToFollow.poses.at(lpIdx-gp_start_index).pose.position.y +=path_cos_th*PATHGAP*(avoidPathIdx-5)*resolution;

          
  // }
  localPathToFollow.poses.push_back(global_path_poses.poses.at(gp_start_index));
   for(int lpIdx = gp_start_index; lpIdx <= gp_end_index; lpIdx++){

          localPathToFollow.poses.push_back(global_path_poses.poses.at(lpIdx));
          localPathToFollow.poses.at(lpIdx-gp_start_index+1).pose.position.x +=path_sin_th*PATHGAP*(avoidPathIdx-3)*resolution;
          localPathToFollow.poses.at(lpIdx-gp_start_index+1).pose.position.y +=path_cos_th*PATHGAP*(avoidPathIdx-3)*resolution;

        
      }
  local_path_pub2.publish(localPathToFollow);

}
void pathPlanner::makeShortAvoidancePath(nav_msgs::Path &global_path_poses,ros::Publisher &local_path_pub2)
{
   if(gp_start_index == -1)
    return;
  int zsei = global_path_poses.poses.size();

  // ROS_INFO("Closest : %d | mid :%d | size %d", gp_start_index,gp_end_index,zsei);

  double gpX1 =  global_path_poses.poses.at(gp_start_index).pose.position.x; 
  double gpX2 =  global_path_poses.poses.at(gp_end_index).pose.position.x; 
  double gpY1 =  global_path_poses.poses.at(gp_start_index).pose.position.y; 
  double gpY2 =  global_path_poses.poses.at(gp_end_index).pose.position.y; 

  nav_msgs::Path localPathToFollow;
  localPathToFollow.header.frame_id = "/map";
  localPathToFollow.header.stamp = plannerTime;
  gp_yaw =  atan2(gpY2-gpY1,gpX2-gpX1);
  ROS_INFO("| %f: %f |",gp_yaw, gp_yaw*M_RAD);

  path_cos_th = cos(180/M_RAD - gp_yaw);
  path_sin_th = sin(180/M_RAD - gp_yaw);
  cos_th_inv = cos(-(yaw));
  sin_th_inv = sin(-(yaw));
  int crash_weight[26]={0}; //(-1.5m, -1.0m, -0.5m, 0m, 0.5m ...)
  double rot_x, rot_y;
  int map_index, local_index;
  double path_x, path_y,rot_path_x,rot_path_y,local_map_x,local_map_y;
  
  for(int path_idx = 1; path_idx <16;path_idx++){
      // 충돌판정 및 선택
       for(int lpIdx = gp_start_index; lpIdx <= gp_end_index; lpIdx++){

        // localPathToFollow.poses.push_back(global_path_poses.poses.at(lpIdx));
  
        path_x = (global_path_poses.poses.at(lpIdx).pose.position.x)+path_sin_th*PATHGAP*(path_idx-12)*resolution; //회전 전 local맵에서의 좌표
        path_y = (global_path_poses.poses.at(lpIdx).pose.position.y)+path_cos_th*PATHGAP*(path_idx-12)*resolution;
       
        path_x = path_x - crrX;
        path_y = path_y - crrY;


        rot_path_x = cos_th_inv * path_x  - sin_th_inv * path_y;
        rot_path_y = sin_th_inv * path_x  + cos_th_inv * path_y;

        local_map_y = (double)((double)yAxis - rot_path_x)/resolution; 
        local_map_x = ((double)xAxis/2.0 - rot_path_y)/resolution;
        int tempIMGdata = local_grid_image.at<unsigned char>((int)local_map_y, (int)local_map_x);

        if(local_map_x > 3.0 && local_map_x<177.0 && local_map_y> 20.0 && local_map_y<130.0){
            int adsad = local_grid_image.at<unsigned char>((int)local_map_y, (int)local_map_x);
            // ROS_INFO("path: %d, data : %d ,X: %lf Y:%lf",path_idx,adsad,local_map_x,local_map_y);
          if(tempIMGdata== 255){
            crash_weight[path_idx-1]+= 100 + (path_idx - avoidPathIdx)*(path_idx - avoidPathIdx)+local_map_y;
            crash_weight[path_idx]+= 150 + (path_idx - avoidPathIdx)*(path_idx - avoidPathIdx)+local_map_y;
            crash_weight[path_idx+1]+= 100 + (path_idx - avoidPathIdx)*(path_idx - avoidPathIdx)+local_map_y;
          }
        }
        else if(local_map_x > 3.0 && local_map_x<177.0 && local_map_y> 130.0 && local_map_y<147.0){
          int adsad = local_grid_image.at<unsigned char>((int)local_map_y, (int)local_map_x);
          ROS_INFO("path: %d, data : %d ,X: %lf Y:%lf",path_idx,adsad,local_map_x,local_map_y);
          if(tempIMGdata== 255){
            if(path_idx<=avoidPathIdx){ //장애물이 왼쪽에 있으면
              for(int weightIdx = 0; weightIdx <=path_idx; weightIdx++){
                crash_weight[weightIdx]+= 10 + (path_idx - avoidPathIdx)*(path_idx - avoidPathIdx);
              }
            }
            else if(path_idx>=avoidPathIdx){ //장애물이 오른쪽에 있으면
              for(int weightIdx =path_idx; weightIdx <14; weightIdx++){
                crash_weight[weightIdx]+= 500 + (path_idx - avoidPathIdx)*(path_idx - avoidPathIdx);
              }
            }
          }
        }
      }
  }

  for(int path_idx = 0; path_idx <16 ;path_idx++){
        crash_weight[path_idx] += abs(path_idx - avoidPathIdx);
    ROS_INFO("| %d: %d |",path_idx, crash_weight[path_idx]);
  }

  int MinObsSum = 999999;//구구구 비둘기야 밥ㅂ머
  for(int path_idx = 1; path_idx <13;path_idx++){
    int avg = (crash_weight[path_idx-1] + crash_weight[path_idx] + crash_weight[path_idx+1])/3;
    if(MinObsSum > avg){
      
      avoidPathIdx = path_idx;
      MinObsSum = avg;
    }
  }
  
  if(crash_weight[11] < 15&&crash_weight[12] < 15&&crash_weight[13] < 15){
    avoidPathIdx = 12;
  }
    ROS_INFO("-------------ShortPATH : %d-------------- ",avoidPathIdx);

  // for(int lpIdx = gp_start_index; lpIdx <= gp_end_index; lpIdx++){

  //         localPathToFollow.poses.push_back(global_path_poses.poses.at(lpIdx));
  //         localPathToFollow.poses.at(lpIdx-gp_start_index).pose.position.x +=path_sin_th*PATHGAP*(avoidPathIdx-5)*resolution;
  //         localPathToFollow.poses.at(lpIdx-gp_start_index).pose.position.y +=path_cos_th*PATHGAP*(avoidPathIdx-5)*resolution;

          
  // }
  localPathToFollow.poses.push_back(global_path_poses.poses.at(gp_start_index));
   for(int lpIdx = gp_start_index; lpIdx <= gp_end_index; lpIdx++){

          localPathToFollow.poses.push_back(global_path_poses.poses.at(lpIdx));
          localPathToFollow.poses.at(lpIdx-gp_start_index+1).pose.position.x +=path_sin_th*PATHGAP*(avoidPathIdx-12)*resolution;
          localPathToFollow.poses.at(lpIdx-gp_start_index+1).pose.position.y +=path_cos_th*PATHGAP*(avoidPathIdx-12)*resolution;

        
      }
  local_path_pub2.publish(localPathToFollow);


}
void pathPlanner::makeLocalCostMap(ros::Publisher &local_costmap_pub,nav_msgs::Path &global_path_poses)
{
  if(gp_start_index == -1)
    return;
   tf::Quaternion q(
                  carpose.pose.orientation.x,
                  carpose.pose.orientation.y,
                  carpose.pose.orientation.z,
                  carpose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  current_th = yaw;
  cos_th = cos(yaw);
  sin_th = sin(yaw);
  double robot_x, robot_y, rot_x, rot_y;
  int map_index, local_index;
  unsigned int map_x, map_y;
  local_map.header.frame_id = "/novatel";
  local_map.info.resolution = resolution;
  local_map.info.height = xAxis/resolution;
  local_map.info.width = yAxis/resolution;
  local_map.info.origin.orientation.w = 1.0;
  local_map.info.origin.position.y = -(double)xAxis/2;
  // local_map.info.origin.position.x =  carpose.pose.position.x-global_map.info.origin.position.x;
  // local_map.info.origin.position.y =  carpose.pose.position.y+global_map.info.origin.position.y;

  
  local_map.data.assign((xAxis/resolution) * (yAxis/resolution), 0);
  local_map.header.stamp = plannerTime;
  local_map.info.map_load_time = plannerTime;
  robot_x = (double)yAxis/resolution;
  robot_y = (double)xAxis/2.0/resolution;
  for (int i = 0; i < local_map.info.width; i++)
  {
      for (int j = 0; j < local_map.info.height; j++)
      {
          rot_x = cos_th * (((double)i - robot_x) * resolution) - sin_th * (((double)j - robot_y) * resolution);
          rot_y = sin_th * (((double)i - robot_x) * resolution) + cos_th * (((double)j - robot_y) * resolution);
          map_x = ((crrX - rot_x) - plain_map.info.origin.position.x) / resolution;
          map_y = ((crrY - rot_y) - plain_map.info.origin.position.y) / resolution;
          map_index = map_y * plain_map.info.width + map_x;
          //ROS_INFO("map_index %d",map_index);
          local_index = local_map.info.width * local_map.info.height - (j * local_map.info.width + i) - 1;
          unsigned int original_map_data;
          if (map_index >= plain_map.info.height * plain_map.info.width)
          {
              original_map_data = -1;
          }
          else
          {
              original_map_data = plain_map.data.at(map_index);
          }
          local_map.data.at(local_index) = original_map_data;
      }
    }
    int lane_boundary_pixel = (int)(lane_boundary);

    cv::Mat temp_map_image(local_map.info.width, local_map.info.height, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < local_map.info.width; i++)
    {
        for (int j = 0; j < local_map.info.height; j++)
        {
            int local_index = local_map.info.width * local_map.info.height - (j * local_map.info.width + i) - 1;
            temp_map_image.at<unsigned char>(i, j) = (int)local_map.data.at(local_index);

            int local_index_lane = local_map.info.width * local_map.info.height - (j * local_map.info.width + i) - 1;
            if ((int)local_map.data.at(local_index_lane) > 20 || (int)local_map.data.at(local_index_lane) == -1)
            {
                cv::circle(temp_map_image, cv::Point(j, i), lane_boundary_pixel, cv::Scalar(255), CV_FILLED, 8);
            }
        }
    }
    for (int i = 0; i < obstacle_pcl.size(); i++)
    {
        float pcl_x = obstacle_pcl.at(i).x +lidar_gps_offset2020;
        float pcl_y = obstacle_pcl.at(i).y;
        // std::cout<<"x:"<<pcl_x<<" y:"<<pcl_y<<"\n"; 
        // std::cout<<"xAxis:"<<xAxis<<" yAxis:"<<yAxis<<"\n\n"; 
        if (pcl_x < (float)xAxis && pcl_x > -(float)xAxis && pcl_y < (float)yAxis && pcl_y > -(float)yAxis)
        {
            int pcl_x_i = ((float)xAxis/2-pcl_y)/resolution;
            int pcl_y_i = ((float)yAxis-pcl_x)/resolution;
            cv::circle(temp_map_image, cv::Point( pcl_x_i,pcl_y_i), obs_boundary, cv::Scalar(255), CV_FILLED, 8);
        }
    }
    // for (int i = 0; i < local_map.info.width; i++)
    // {
    //     for (int j = 0; j < local_map.info.height; j++)
    //     {
    //         int local_index_image = local_map.info.width * local_map.info.height - (j * local_map.info.width + i) - 1;
    //         local_map.data.at(local_index_image) = (int)temp_map_image.at<unsigned char>(i, j);
    //     }
    // }
  double gpX1 =  global_path_poses.poses.at(gp_start_index).pose.position.x; 
  double gpX2 =  global_path_poses.poses.at(gp_end_index).pose.position.x; 
  double gpY1 =  global_path_poses.poses.at(gp_start_index).pose.position.y; 
  double gpY2 =  global_path_poses.poses.at(gp_end_index).pose.position.y; 

  nav_msgs::Path localPathToFollow;
  localPathToFollow.header.frame_id = "/map";
  localPathToFollow.header.stamp = plannerTime;
  gp_yaw =  atan2(gpY2-gpY1,gpX2-gpX1);
  path_cos_th = cos(gp_yaw);
  path_sin_th = sin(gp_yaw);


  cos_th_inv = cos(-(yaw));
  sin_th_inv = sin(-(yaw));
  int crash_weight[5]={0};
  double path_x, path_y,rot_path_x,rot_path_y,local_map_x,local_map_y;
    // for(int lpIdx = gp_start_index; lpIdx <= gp_end_index; lpIdx++){

    //           localPathToFollow.poses.push_back(global_path_poses.poses.at(lpIdx));
              
      
    //           path_x = (localPathToFollow.poses.at(lpIdx-gp_start_index).pose.position.x); //회전 전 local맵에서의 좌표
    //           path_y = (localPathToFollow.poses.at(lpIdx-gp_start_index).pose.position.y);


    //           path_x = path_x - crrX;
    //           path_y = path_y - crrY;

    //           // std::cout<<"path_xoffset:"<<path_x<<"| path_yoffset:"<<path_y<<std::endl;

    //           rot_path_x = cos_th_inv * path_x  - sin_th_inv * path_y;
    //           rot_path_y = sin_th_inv * path_x  + cos_th_inv * path_y;

    //           // std::cout<<"rot_path_x:"<<rot_path_x<<"| rot_path_y:"<<rot_path_y<<std::endl;
    //           // std::cout<<"local_map.info.height:"<<local_map.info.height<<"| rot_path_y:"<<rot_path_y<<std::endl;

    //           local_map_y = (double)(yAxis - rot_path_x)/resolution; 
    //           local_map_x = (xAxis/2.0 - rot_path_y)/resolution;
    //           // std::cout<<"local_map_x:"<<local_map_x<<"| local_map_y:"<<local_map_y<<std::endl;

    //           if(local_map_x > 0.0 && local_map_x<180.0 && local_map_y> 0.0 && local_map_y<150.0){
    //             // int local_index_image = local_map.info.width * local_map.info.height - (local_map_x* local_map.info.width + local_map_y) - 1;
    //             // local_map.data.at(local_index_image) = 100;
    //             unsigned char tempIMGdata = temp_map_image.at<unsigned char>((int)local_map_y, (int)local_map_x);
    //             if(tempIMGdata== 255){
    //                 cv::circle(temp_map_image, cv::Point( local_map_x,local_map_y), 2, cv::Scalar(150), CV_FILLED, 8);

    //             }
    //             else{
    //              cv::circle(temp_map_image, cv::Point( local_map_x,local_map_y), 2, cv::Scalar(50), CV_FILLED, 8);
    //             }
    //           }
    //   }
          temp_map_image.copyTo(local_grid_image);

    for (int i = 0; i < local_map.info.width; i++)
      {
        for (int j = 0; j < local_map.info.height; j++)
        {
            int local_index_image = local_map.info.width * local_map.info.height - (j * local_map.info.width + i) - 1;
            local_map.data.at(local_index_image) = (int)local_grid_image.at<unsigned char>(i, j);
            
        }
      }

  local_costmap_pub.publish(local_map);
}
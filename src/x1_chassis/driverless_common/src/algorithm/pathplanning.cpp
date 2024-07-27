#include "driverless_common/algorithm/pathplanning.h"
#include <algorithm>
#include <climits>

namespace dcom {

GlobalPath::Ptr optimalPathPlanning(const GlobalPath::Ptr in_path, const VehicleState::ConstPtr v_state,
                                    const std::vector<Pose>& way_pts){
  const Pose ego_pose = v_state->pose;
  std::vector<size_t> order(way_pts.size());
  for(size_t i=0; i<order.size(); ++i){
    order[i] = i;
  }

  auto getDistance = [&](const std::vector<size_t>& _order){
      float distance = 0.0;
      Pose last_pose = ego_pose;
      for(size_t i=0; i<_order.size(); ++i){
          distance += way_pts[_order[i]].disTo(last_pose);
          last_pose = way_pts[_order[i]];
      }
      return distance;
  };

  std::vector<size_t> optimal_order = order;
  float min_distance = getDistance(order);
  // printVector(order, "order");
  // std::cout << "distance: " << min_distance << std::endl;
  while(std::next_permutation(order.begin(), order.end())){
    float distance = getDistance(order);
    if(distance < min_distance){
      min_distance = distance;
      optimal_order = order;
    }
//    printVector(order, "order");
//    std::cout << "distance: " << distance << std::endl;
  }

  printVector(optimal_order, "optimal_order");
  std::cout << "min_distance: " << min_distance << std::endl;

  GlobalPath::Ptr out_path = std::make_shared<GlobalPath>();
  out_path->clear();
  float resolution = 0.1f;

  Pose start_pose = ego_pose;
  for(size_t i=0; i<optimal_order.size(); ++i){
    const Pose& end_pose = way_pts[optimal_order[i]];
    std::vector<Pose> points = interpolate(start_pose, end_pose, resolution);

    for(const Pose& point : points){
      GlobalPathPoint::Ptr path_point = std::make_shared<GlobalPathPoint>();
      path_point->x = point.x;
      path_point->y = point.y;
      path_point->yaw = point.yaw;
      path_point->curvature = 0.0f;
      path_point->left_width = 1.5f;
      path_point->right_width = 1.5f;
      out_path->points.push_back(path_point);
    }

    /* 根据两条路线的夹角θ，估算夹角处的路径曲率，以限制车速
     * 做运动圆与两条路线相切，
     * 为确保行驶轨迹不过分偏离途经点，圆弧到途经点的距离d， 应不小于期望值t_d
     * 当d=t_d时，可根据R/(R+d)=sin(θ/2) 求得 R = d*sin(θ/2)/(1-sin(θ/2))
     * 进而, 曲率ρ = 1/R = (1-sin(θ/2))/d/sin(θ/2) = (1/d)(1/sin(θ/2)-1)
     * 也就是说, 根据t_d可推算曲率，t_d越小曲率越大,反之曲率越小
     */
    if(i<optimal_order.size()-1){
      // 计算夹角
      float yaw1 = getYaw(start_pose, way_pts[optimal_order[i]]);
      float yaw2 = getYaw(way_pts[optimal_order[i]], way_pts[optimal_order[i+1]]);
//      float theta = yaw2 - yaw1;
//      if(theta < 0){
//          theta += M_PI;
//      }
//      if(theta > 180){
//        theta = 2*M_PI - theta;
//      }

      float a = start_pose.disTo(way_pts[optimal_order[i]]);
      float b = way_pts[optimal_order[i]].disTo(way_pts[optimal_order[i+1]]);
      float c = start_pose.disTo(way_pts[optimal_order[i+1]]);
      float theta = acos((a*a+b*b-c*c)/(2*a*b));

      float d = 0.3f;
      float rho = 1.0f/d * (1.0f/sin(theta/2)-1);

      printf("[optimalPathPlanning] yaw1: %.1f, yaw2: %.1f, theta: %.1f, rho: %.2f\r\n",
             dcom::rad2deg(yaw1), dcom::rad2deg(yaw2), dcom::rad2deg(theta), rho);

      out_path->points.back()->curvature = rho;
    }

    start_pose = end_pose;

    // 配送点处是否需要插入停车点?
  }

  out_path->resolution = resolution;
  out_path->has_curvature = true;
  out_path->final_index = out_path->points.size()-1;
  out_path->offsets.resize(out_path->points.size(), 0.0f);

  out_path->park_points.insert(ParkingPoint(out_path->final_index, -1));

  extendPath(*out_path, 20.0f);

  return out_path;
}












}//end namespace dcom

#ifndef DRIVERLESS_COMMON_UTILS_H_
#define DRIVERLESS_COMMON_UTILS_H_

#include <cstring>
#include <cmath>
#include <assert.h>
#include <string>
#include <vector>
#include <cstdio>
#include <ros/ros.h>
#include <limits.h>
#include <exception>
#include <fstream>
#include <tinyxml2.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPoint.h>
#include <driverless_common/structs.h>

namespace dcom{

/* @brief 从ros参数服务器载入车辆参数
 * @param params 车辆参数
 * @param loader 载入者
 * @return 载入结果
 */
bool loadVehicleParams(VehicleParams& params, const std::string& loader);

/*@brief 角度归一化，(-pi, pi]
 */
double normalizeRadAngle(double angle);

/* @brief 零点处理,当数据在零点附近时,使用较小值进行代替,防止除0错误
 * @param input 输入数据
 * @param clamp_val 最小值
 */
float zeroPointClamp(float input, float clamp_val);

double deg2rad(double deg);
double rad2deg(double rad);

/* @brief 归一化航向偏差
 * @param yaw1 航向1
 * @param yaw2 航向2
 */
double yawDeviation(double yaw1, double yaw2);

int min_int(int a, int b);
int max_int(int a, int b);

float min_float(float a, float b);
float max_float(float a, float b);

/*@brief 获取两点间的距离以及航向
 *@param point1 终点
 *@param point2 起点
 */
std::pair<float, float> getDisAndYaw(const Pose& point1, const Pose& point2);

double max(double val_1, double val_2);

/*@brief 获取两点间的航向
 *@param point1 终点
 *@param point2 起点
 */
float getYaw(const Point& point1, const Point& point2);


/*@brief 获取两点间的距离
 *@param point1 终点
 *@param point2 起点
 */
float getDistance(const Point& point1, const Point& point2);


/*@brief 计算路径各离散点处的曲率
 *@param path 引用原始路径
 */
template <typename PointT>
bool calPathCurvature(Path<PointT> &path){
    if(path.has_curvature) //路径已经包含曲率信息，无需重复计算
        return true;
    auto& points = path.points;
    size_t size = points.size();
    for(int i=0; i<size-1; ++i){
        float delta_theta = normalizeRadAngle(points[i+1]->yaw - points[i]->yaw); //旋转角
        float arc_length  = getDistance(*points[i+1], *points[i]); //利用两点间距近似弧长
        if(arc_length == 0)
            points[i]->curvature = 0.0; //绝对值偏大
        else
            points[i]->curvature = delta_theta/arc_length; //绝对值偏大
    }

    //均值滤波
    int n = 10;
    float curvature_n_sum = 0.0;
    for(int i=0; i < size; ++i){
        if(i<n)
            curvature_n_sum+=points[i]->curvature;
        else{
            points[i-n/2]->curvature = curvature_n_sum/n;
            curvature_n_sum += (points[i]->curvature - points[i-n]->curvature);
        }
    }

    path.has_curvature = true;
    return true;
}

/* @brief 从文件载入路径点,包括位置，航向，以及路径曲率
 *       若文件中不包含曲率信息，则调用曲率计算函数进行计算
 * @param file_path 文件路径
 * @param path 最终路径
 * @param set_cycle 设置为循环路径
 * @param extend 是否拓展路径
 * @return false: 载入失败
 */
bool loadGlobalPathPoints(const std::string& file_path, GlobalPath& path, bool set_cycle=false, bool extend=true);

/*@brief 检查xml文件有效性
 */
bool checkXmlFile(const std::string& file, const std::string& user="checkXmlFile");

/*@brief 从xml文件载入路径停车点信息
 */
bool loadNavParkingPointsInfo(const std::string& file, ParkingPoints& park_points,
                              size_t final_idx, bool cycle=false, const std::string& user="");


/*@brief 从xml文件载入路径转向区间信息
 */
bool loadNavTurnRangesInfo(const std::string& file, TurnRanges& turn_ranges, const std::string& user="");
/*@brief 从xml文件载入路径站点信息
 */
bool loadNavStationPointsInfo(const std::string& file, StationPoints& station_points,
                                     const std::string& user="loadNavStationPointsInfo");

/*@brief 从xml文件载入路径信息
 *@1. 停车点-若文件不包含终点信息，手动添加√
 *@2. 转向区间-控制转向灯　
 *@3. 站点-报站
 *@?. 目前未对索引的有效性进行判断
*/
bool loadPathAppendInfos(const std::string& file, GlobalPath& global_path,
                         bool cycle=false, const std::string& user="common_utils");

/*@brief 拓展路径,防止车辆临近终点时无法预瞄
 *@param path  需要拓展的路径
 *@param extendDis 拓展长度，保证实际终点距离虚拟终点大于等于extendDis
 */
bool extendPath(GlobalPath& path, float extendDis);

/* @brief 饱和函数
 * @param value 输入值
 * @param limit 限幅
 * @return 限幅后的结果
 */
float saturation(float value,float limit);

/* @brief 计算两点距离
 * @param is_sqrt 默认开方
 */
float dis2Points(const Pose& point1, const Pose& point2,bool is_sqrt=true);
float disBetweenPoints(const Point &point1, const Point &point2);


/*@brief 计算目标点到达路径的距离,点在路径左侧为负,右侧为正
 *@brief 该函数主要用于计算主体车辆到路径的距离(横向偏差),并更新最近点索引
 *@param x,y         目标点坐标
 *@param path        路径点集
 *@param ref_point_index 参考点索引，以此参考点展开搜索，加速计算
 *@param nearest_point_index_ptr 输出与目标点最近的路径点索引(可选参数)
 */
float calculateDis2path(const double& x,const double& y,
						 const GlobalPath& path, 
						 size_t   ref_point_index, //参考点索引
						 size_t * const nearest_point_index_ptr=NULL);

/*@brief 计算目标点到达路径的距离,点在路径左侧为负,右侧为正
 *@brief 该函数主要用于计算目标到路径的距离,并考虑路径终点问题
 *@param x,y         目标点坐标
 *@param path        路径
 *@param ref_point_index 参考点索引，以此参考点展开搜索，加速计算
 *@param max_search_index 最大搜索索引,超出此索引的目标物则输出距离为FLT_MAX
 */
float calculateDis2path(const double& x,const double& y,
						 const GlobalPath& path, 
						 size_t  ref_point_index, //参考点索引
						 size_t  max_search_index);

/* @brief 计算点到射线的距离
 * @param ray 射线方程(x, y, yaw)
 * @param point 参考点(x, y)
 */
float dis2ray(const Pose& ray, const Point& point);

/*@brief 查找与当前点距离为dis的路径点索引
 *@param path_points 路径点集
 *@param startIndex  搜索起点索引
 *@param dis         期望距离
*/
size_t findIndexForGivenDis(const std::vector<GpsPathPoint>& path_points,
							size_t startIndex,float dis);
float minCurvatureInRange(const std::vector<GpsPathPoint>& path_points, size_t startIndex,size_t endIndex);


/*@brief 搜索从startIndex开始到dis距离区间的最大曲率
 *@brief剩余距离小于期望距离时输出剩余部分的最大曲率
*/
template<typename T>
std::pair<float, float> maxCurvatureInRange(const Path<T>& path, size_t startIndex,float dis)
{
	float sum_dis = 0.0;
	float max_cuvature = 0.0;
	float max_curvature_dis = 0.0; // 最大曲率对应的距离
	
	float now_cuvature;
    const auto& path_points = path.points;
	for(size_t i =startIndex; i<path_points.size()-1; ++i)
	{
        now_cuvature = fabs(path_points[i]->curvature);
		if(max_cuvature < now_cuvature){
		    max_cuvature = now_cuvature;
		    max_curvature_dis = sum_dis;
		}
        sum_dis	+= disBetweenPoints(*path_points[i], *path_points[1+i]);
		if(sum_dis >= dis)
			break;
	}
	return std::make_pair(max_cuvature, max_curvature_dis);
}

template<typename T>
float maxCurvatureInRange(const Path<T>& path, size_t startIndex,size_t endIndex){
	float max = 0.0;
    const auto& path_points = path.points;
	for(size_t i=startIndex; i<endIndex; i++)
	{
        if(fabs(path_points[i]->curvature) > max)
            max = fabs(path_points[i]->curvature);
	}
	return max;
}



/*@brief local2global坐标变换
 *@param origin_x,origin_y   局部坐标系在全局坐标下的位置
 *@param theta 局部坐标系在全局坐标下的角度
 *@param local_x,local_y   点在局部坐标系下的坐标
 *@return      点在全局坐标系下的坐标
 */
std::pair<double, double> 
local2global(double origin_x,double origin_y,float theta, float local_x,float local_y);

/*@brief global2local坐标变换
 *@param origin_x,origin_y   局部坐标系在全局坐标下的位置
 *@param theta 局部坐标系在全局坐标下的角度
 *@param local_x,local_y   点在局部坐标系下的坐标
 *@return      点在全局坐标系下的坐标
 */
std::pair<float, float> 
global2local(double origin_x,double origin_y,float theta, double global_x,double global_y);

/*@brief 坐标变换
 *@param origin   局部坐标系原点在全局坐标下的位姿
 *@param local   点在局部坐标系下的位置
 *@return        点在全局坐标系下的位置
 */
Point local2global(const Pose& origin, const Point& local);
Pose local2global(const Pose& origin, const Pose& local);

Point global2local(const Pose& origin, const Point& global);

/*@brief 全局坐标系转局部坐标系
 *@param origin   局部坐标系原点在全局坐标下的位姿
 *@param global   在全局坐标系下的位姿
 *@return         在局部坐标系下的位姿
 */
Pose global2local(const Pose& origin, const Pose& global);

int sign(float val);
/* @brief 根据两个端点,计算特定x所对应的y值, 当x不在x1,x2范围内,取饱和值
 */
float computeYby2ends(float x1, float y1, float x2, float y2, float x);

bool isZeroFloat(float value);
bool isZeroDouble(double value);


/* @brief 路径点偏移
 * @param pose 需要偏移的路径点
 * @param offset 偏移距离(沿局部系y轴)
 */
template <typename poseT>
poseT poseOffset(const poseT& pose,float offset){
    poseT result = pose;
    result.x =  -offset * sin(pose.yaw) + pose.x;
    result.y = offset * cos(pose.yaw) + pose.y;
    return result;
}


/* @brief 路径点偏移
 * @param pose 需要偏移的路径点
 * @param offset 偏移距离(沿局部系y轴)
 */
template <typename poseT>
std::shared_ptr<poseT> posePtrOffset(const std::shared_ptr<poseT>& pose_in, float offset){
    std::shared_ptr<poseT> result = std::make_shared<poseT>(*pose_in);
    result->x = -offset * sin(pose_in->yaw) + pose_in->x;
    result->y = offset * cos(pose_in->yaw) + pose_in->y;
    return result;
}

template <typename T>
void printVector(const std::vector<T>& array, const std::string& prefix=""){
  if(!prefix.empty()){
    std::cout << prefix << ":\t";
  }
  for(const T& a : array){
    std::cout << a << " ";
  }
  std::cout << std::endl;
}

std::vector<Pose> interpolate(const Pose& p1, const Pose& p2, float space);
LocalPath::Ptr offsetToNewPath(const LocalPath::Ptr in_path, float offset);

geodesy::UTMPoint toUtm(double longitude, double latitude, double altitude=0.0);


}//end namespace dcom
#endif


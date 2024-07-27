#include <driverless_common/utils.h>

namespace dcom{

/* @brief 从ros参数服务器载入车辆参数
 * @param params 车辆参数
 * @param loader 载入者
 * @return 载入结果
 */
bool loadVehicleParams(VehicleParams& params, const std::string& loader){
    bool ok = true;
    ros::NodeHandle nh("vehicle");
    params.max_roadwheel_angle = nh.param<float>("max_roadwheel_angle",0.0f);
    params.min_roadwheel_angle = nh.param<float>("min_roadwheel_angle",0.0f);
    params.min_steering_radius          = nh.param<float>("min_steering_radius",0.0f);
    params.wheel_base = nh.param<float>("wheel_base",0.0f);
    params.wheel_track = nh.param<float>("wheel_track",0.0f);
    params.width = nh.param<float>("width",0.0f);
    params.length = nh.param<float>("length",0.0f);
    params.max_deceleration = nh.param<float>("max_deceleration", 0.0f);
    params.max_acceleration = nh.param<float>("max_acceleration", 0.0f);
    params.max_side_acceleration = nh.param<float>("max_side_acceleration", 0.0f);
    params.speed_control_period = nh.param<float>("speed_control_period", 0.0f);
    params.steer_control_period = nh.param<float>("steer_control_period", 0.0f);
    params.max_speed = nh.param<float>("max_speed",0.0f);
    params.min_speed = nh.param<float>("min_speed",0.0f);
    params.max_steering_speed = nh.param<float>("max_steering_speed", 0.0f);
    params.steer_clearance = nh.param<float>("steer_clearance", 0.0f);

    auto checkParam = [&](float param, const std::string& param_name, float error_value){
        if(param == error_value){
            ROS_ERROR("[%s] No parameter %s.",loader.c_str(), param_name.c_str());
            ok = false;
        }
    };

    checkParam(params.max_side_acceleration, "max_side_acceleration", 0.0f);

    ROS_INFO("[%s] Loading vehicle parameters...", loader.c_str());
    if(params.max_roadwheel_angle == 0.0f){
        ROS_ERROR("[%s] No parameter max_roadwheel_angle.",loader.c_str()); ok = false;
    }
    if(params.min_roadwheel_angle == 0.0f){
        ROS_ERROR("[%s] No parameter min_roadwheel_angle.",loader.c_str()); ok = false;
    }
    if(params.min_steering_radius == 0.0f){
        ROS_ERROR("[%s] No parameter min_steering_radius.",loader.c_str()); ok = false;
    }
    if(params.wheel_base == 0.0f){
        ROS_ERROR("[%s] No parameter wheel_base.",loader.c_str()); ok = false;
    }
    if(params.wheel_track == 0.0f){
        ROS_ERROR("[%s] No parameter wheel_track.",loader.c_str()); ok = false;
    }
    if(params.width == 0.0f){
        ROS_ERROR("[%s] No parameter width.",loader.c_str()); ok = false;
    }
    if(params.length == 0.0f){
        ROS_ERROR("[%s] No parameter length.",loader.c_str()); ok = false;
    }
    if(params.max_deceleration == 0.0f){
        ROS_ERROR("[%s] No parameter max_deceleration.",loader.c_str()); ok = false;
    }
    if(params.max_acceleration == 0.0f){
        ROS_ERROR("[%s] No parameter max_acceleration.",loader.c_str()); ok = false;
    }
    if(params.speed_control_period == 0.0f){
        ROS_ERROR("[%s] No parameter speed_control_period.",loader.c_str()); ok = false;
    }
    if(params.steer_control_period == 0.0f){
        ROS_ERROR("[%s] No parameter steer_control_period.",loader.c_str()); ok = false;
    }
    if(params.max_speed == 0.0f){
        ROS_ERROR("[%s] No parameter max_speed.",loader.c_str()); ok = false;
    }
    if(params.min_speed == 0.0f){
        ROS_ERROR("[%s] No parameter min_speed.",loader.c_str()); ok = false;
    }
    if(params.max_steering_speed == 0.0f){
        ROS_ERROR("[%s] No parameter max_steering_speed.",loader.c_str()); ok = false;
    }

    if(ok){
        params.validity = true;
    }else{
        params.validity = false;
        ROS_ERROR("[%s] Load vehicle parameters failed!", loader.c_str());
    }
    ROS_INFO("[%s] Load vehicle parameters succeed.", loader.c_str());
    return ok;
}

// 将所有函数声明为静态函数,防止头文件重复包含导致的函数重复定义报错
// 未使用static修饰的函数或变量具有全局可见性, 
// 当a.cpp 和 b.cpp均包含此头文件时, 将分别引入函数的定义
// 若未使用static修饰, a和b中的函数相互可见而导致重定义
// 使用static修饰后, 互不可见, 互不干涉

/*@brief 角度归一化，(-pi, pi]
 */
double normalizeRadAngle(double angle){
    double res = angle;
    while(res <= -M_PI)
        res += 2*M_PI;

    while(res > M_PI)
        res -= 2*M_PI;
    return res;
}

/* @brief 零点处理,当数据在零点附近时,使用较小值进行代替,防止除0错误
 * @param input 输入数据
 * @param clamp_val 最小值
 */
float zeroPointClamp(float input, float clamp_val){
    clamp_val = fabs(clamp_val);
    if(input > 0 && input < clamp_val){
        return clamp_val;
    }else if(input < 0 && input > -clamp_val){
        return -clamp_val;
    }else{
        return input;
    }
}

/* @brief 归一化航向偏差(-pi, pi]
 * @param yaw1 航向1
 * @param yaw2 航向2
 */
double yawDeviation(double yaw1, double yaw2){
    double yaw_err = yaw1 - yaw2;
    while(yaw_err <= -M_PI){
        yaw_err += 2*M_PI;
    }

    while(yaw_err > M_PI){
        yaw_err -= 2*M_PI;
    }
    return yaw_err;
}

double deg2rad(double deg){
    return deg * 0.017453292519943295;
    //return deg/180.0f*M_PI;
}

double rad2deg(double rad){
    return rad * 57.29577951308232;
}

int min_int(int a, int b){
    return a < b ? a : b;
}

int max_int(int a, int b){
    return a > b ? a : b;
}

float min_float(float a, float b){
    return a < b ? a : b;
}

float max_float(float a, float b){
    return a > b ? a : b;
}

/*@brief 获取两点间的距离以及航向
 *@param point1 终点
 *@param point2 起点
 */
std::pair<float, float> getDisAndYaw(const Pose& point1, const Pose& point2)
{
    float x = point1.x - point2.x;
    float y = point1.y - point2.y;

    std::pair<float, float> dis_yaw;
    dis_yaw.first = sqrt(x * x + y * y);
    dis_yaw.second = atan2(y,x);

    if(dis_yaw.second <0)
        dis_yaw.second += 2*M_PI;
    return dis_yaw;
}

double max(double val_1, double val_2)
{
    return val_1 > val_2 ? val_1 : val_2;
}

/*@brief 获取两点间的航向
 *@param point1 终点
 *@param point2 起点
 */
float getYaw(const Point& point1, const Point& point2)
{
    float yaw = atan2(point1.y - point2.y, point1.x - point2.x);

    if(yaw <0) yaw += 2*M_PI;
    return yaw;
}

/*@brief 获取两点间的距离
 *@param point1 终点
 *@param point2 起点
 */
float getDistance(const Point& point1, const Point& point2)
{
    float x = point1.x - point2.x;
    float y = point1.y - point2.y;
    return sqrt(x * x + y * y);
}

/* @brief 从文件载入路径点,包括位置，航向，以及路径曲率
 *       若文件中不包含曲率信息，则调用曲率计算函数进行计算
 * @param file_path 文件路径
 * @param path 最终路径
 * @param set_cycle 设置为循环路径
 * @param extend 是否拓展路径
 * @return false: 载入失败
 */
bool loadGlobalPathPoints(const std::string& file_path, GlobalPath& path, bool set_cycle, bool extend){
    std::ifstream in_file(file_path.c_str());
    if(!in_file.is_open()){
        ROS_ERROR("LoadPathPoints: Open %s failed",file_path.c_str());
        return false;
    }

    path.clear();  //首先清除历史路径点信息

    bool has_curvature = false;
    bool first_line = true;
    std::string line;
    while(in_file.good()){
        getline(in_file,line);

        //若首行为title, 则跳过
        if(first_line)
        {
            first_line = false;
            if(line.substr(0,5) == "title")
                continue;
        }

        if(line.length() == 0)  //处理当文件为空或者末尾空行的情况
            break;

        GpsPathPoint::Ptr point(new GpsPathPoint);
        std::stringstream ss(line);
        ss >> point->x >> point->y >> point->yaw >> point->curvature >> point->left_width >> point->right_width
                >> point->longitude >> point->latitude;

        if(!has_curvature && point->curvature!=0)
            has_curvature = true;
        path.points.push_back(point);
    }
    in_file.close();

    if(path.points.size() == 0)
        return false;

    float resolution = 0.1;   //实则应从文件读取
    path.resolution  = resolution;
    path.has_curvature = has_curvature;
    if(!has_curvature)
        calPathCurvature(path);

    path.final_index = path.points.size() - 1 ;    //设置终点索引为最后一个点
    path.offsets.assign(path.points.size(), 0.0f); //设置所有点默认偏移为0.
    if(set_cycle){
        return path.setCirularEnable();
    }else{
        if(extend){
            return extendPath(path, 20.0f);
        }
    }
    return true;
}

/*@brief 检查xml文件有效性
 */
bool checkXmlFile(const std::string& file, const std::string& user){

    tinyxml2::XMLDocument Doc;
    tinyxml2::XMLError res = Doc.LoadFile(file.c_str());

    if(tinyxml2::XML_ERROR_FILE_NOT_FOUND == res)
    {
        ROS_ERROR_STREAM("[" << user <<"] " << file << " not exist!");
        return false;
    }
    else if(tinyxml2::XML_SUCCESS != res)
    {
        ROS_ERROR_STREAM("[" << user <<"] " << file << " parse error!");
        return false;
    }
    tinyxml2::XMLElement *pRoot=Doc.RootElement();//根节点
    if(pRoot == nullptr)
    {
        ROS_ERROR_STREAM("[" << user <<"] " << file << " no root node!");
        return false;
    }
    return true;
}

/* @brief 从xml文件载入路径停车点信息
 * @param cycle is cycle path
 */
bool loadNavParkingPointsInfo(const std::string& file, ParkingPoints& park_points,
                              size_t final_idx, bool cycle, const std::string& user){
    if(!checkXmlFile(file, user)){
        return false;
    }

    tinyxml2::XMLDocument Doc;
    tinyxml2::XMLError res = Doc.LoadFile(file.c_str());
    tinyxml2::XMLElement *pRoot=Doc.RootElement();//根节点

    tinyxml2::XMLElement *pParkingPoints = pRoot->FirstChildElement("ParkingPoints"); //一级子节点
    if(pParkingPoints){
        std::vector<ParkingPoint> points;

        bool has_dst_parking_point = false;//是否有终点
        tinyxml2::XMLElement *pParkingPoint = pParkingPoints->FirstChildElement("ParkingPoint"); //二级子节点
        while (pParkingPoint){
            int32_t id    = pParkingPoint->Unsigned64Attribute("id");
            uint32_t index = pParkingPoint->Unsigned64Attribute("index");
            float duration = pParkingPoint->FloatAttribute("duration");
            if(index > final_idx){
              std::cerr << "[loadNavParkingPointsInfo] global_path Do Not Match " << file << std::endl;
              std::cerr << "[loadNavParkingPointsInfo] Invalid parking index: " << index << std::endl;
              assert(index > final_idx);
            }

            if(cycle && duration < 0){ //循环路径忽略终点
                // do nothing
            }else{
                points.emplace_back(index,duration,id);

                if(duration < 0)
                    has_dst_parking_point = true;
            }
            //转到下一子节点
            pParkingPoint = pParkingPoint->NextSiblingElement("ParkingPoint");
        }

        //如果路径信息中不包含终点停车点，手动添加路径终点为停车点
        if(!cycle && !has_dst_parking_point){
            points.emplace_back(final_idx, -1);
            
        }

        park_points.swap(points);
        park_points.sort();  //停车点小到大排序
        park_points.print(user); //打印到终端显示
        
        ROS_INFO("[%s] xml: %s" ,user.c_str(), file.c_str());
        ROS_INFO("[%s] load Parking Points ok.",user.c_str());
    }
    else
        ROS_INFO("[%s] No Parking Points in path info file!",user.c_str());
    return true;
}

/*@brief 从xml文件载入路径转向区间信息
 */
bool loadNavTurnRangesInfo(const std::string& file, TurnRanges& turn_ranges, const std::string& user){
    if(!checkXmlFile(file, user)){
        return false;
    }

    tinyxml2::XMLDocument Doc;
    tinyxml2::XMLError res = Doc.LoadFile(file.c_str());
    tinyxml2::XMLElement *pRoot=Doc.RootElement();//根节点

    tinyxml2::XMLElement *pTurnRanges = pRoot->FirstChildElement("TurnRanges"); //一级子节点
    if(pTurnRanges)
    {
        tinyxml2::XMLElement *pTurnRange = pTurnRanges->FirstChildElement("TurnRange"); //二级子节点
        while (pTurnRange)
        {
            int    type  = pTurnRange->IntAttribute("type");
            size_t start = pTurnRange->Unsigned64Attribute("start");
            size_t end   = pTurnRange->Unsigned64Attribute("end");
            turn_ranges.ranges.emplace_back(type,start,end);
            //std::cout << type << "\t" << start << "\t" << end << std::endl;

            //转到下一子节点
            pTurnRange = pTurnRange->NextSiblingElement("TurnRange");
        }
        for(auto &range : turn_ranges.ranges)
            ROS_INFO("[%s] turn range: type:%d  start:%lu  end:%lu", user.c_str(), range.type,range.start_index, range.end_index);

        ROS_INFO("[%s] load turn ranges ok.",user.c_str());
    }
    else
        ROS_INFO("[%s] No tutn ranges in path info file!",user.c_str());
    return true;
}

/*@brief 从xml文件载入路径站点信息
 */
bool loadNavStationPointsInfo(const std::string& file, StationPoints& station_points,
                              const std::string& user){
    if(!checkXmlFile(file, user)){
        return false;
    }

    tinyxml2::XMLDocument Doc;
    tinyxml2::XMLError res = Doc.LoadFile(file.c_str());
    tinyxml2::XMLElement *pRoot=Doc.RootElement();//根节点

    // StationPoints 站点信息
    tinyxml2::XMLElement *pStationPoints = pRoot->FirstChildElement("StationPoints"); //一级子节点
    if(pStationPoints){
        tinyxml2::XMLElement *pStationPoint = pStationPoints->FirstChildElement("StationPoint"); //二级子节点
        while (pStationPoint){
            int    id  = pStationPoint->IntAttribute("id");
            std::string name = pStationPoint->Attribute("name");
            size_t index = pStationPoint->Unsigned64Attribute("index");
            bool is_stay = pStationPoint->BoolAttribute("is_stay");
            int stay_duration = pStationPoint->IntAttribute("stay_duration");
            station_points.points.emplace_back(id, name, index, is_stay, stay_duration);

            //std::cout << type << "\t" << start << "\t" << end << std::endl;
            //转到下一子节点
            pStationPoint = pStationPoint->NextSiblingElement("StationPoint");
        }

        station_points.sort();  //按索引从小到大排序
        station_points.print(user); //打印到终端显示
        ROS_INFO("[%s] load station points ok.",user.c_str());
    }else
        ROS_INFO("[%s] No station points in path info file!", user.c_str());
    return true;
}

/* @brief 从xml文件载入路径信息
 * @param cycle is cycle path
 * @1. 停车点-若文件不包含终点信息，手动添加√
 * @2. 转向区间-控制转向灯
 * @3. 站点-报站
 * @?. 目前未对索引的有效性进行判断
*/
bool loadPathAppendInfos(const std::string& file, GlobalPath& global_path,
                         bool cycle, const std::string& user)
{
    if(global_path.size() == 0){
        ROS_ERROR("[%s] please load global path points first!",user.c_str());
        return false;
    }

    ParkingPoints& park_points = global_path.park_points;
    TurnRanges&    turn_ranges = global_path.turn_ranges;
    StationPoints& station_points = global_path.station_points;

    return (loadNavParkingPointsInfo(file, park_points, global_path.final_index, cycle, user) &&
            loadNavTurnRangesInfo(file, turn_ranges, user) &&
            loadNavStationPointsInfo(file, station_points, user)
            );
}

/*@brief 拓展路径,防止车辆临近终点时无法预瞄
 *@param path  需要拓展的路径
 *@param extendDis 拓展长度，保证实际终点距离虚拟终点大于等于extendDis
 */
bool extendPath(GlobalPath& path, float extendDis)
{
    auto& path_points = path.points;
    auto& path_offsets = path.offsets;
    //取最后一个点与倒数第n个点的连线向后插值
    //总路径点不足n个,退出
    int n = 5;
    //std::cout << "extendPath: " << path_points.size() << "\t" << path_points.size()-1 << std::endl;
    if(path_points.size()-1 < n){
        ROS_ERROR("path points is too few (%lu), extend path failed",path_points.size()-1);
        return false;
    }
    int endIndex = path_points.size()-1;

    float dx = (path_points[endIndex]->x - path_points[endIndex-n]->x)/n;
    float dy = (path_points[endIndex]->y - path_points[endIndex-n]->y)/n;
    float ds = sqrt(dx*dx+dy*dy);

    float yaw = atan2(dy, dx);
    float remaind_dis = 0.0f;
    for(size_t i=1;;++i){
        GpsPathPoint::Ptr point(new GpsPathPoint);
        point->x = path_points[endIndex]->x + dx*i;
        point->y = path_points[endIndex]->y + dy*i;
        point->yaw = yaw; //XXX
        point->curvature = 0.0f;
        path_points.push_back(point);
        path_offsets.push_back(0.0f);
        remaind_dis += ds;
        if(remaind_dis > extendDis)
            break;
    }
    return true;
}

/* @brief 饱和函数
 * @param value 输入值
 * @param limit 限幅
 * @return 限幅后的结果
 */
float saturation(float value,float limit)
{
    assert(limit>=0);
    if(value > limit) value = limit;
    else if(value < -limit) value = -limit;
    return value;
}

/* @brief 计算两点距离
 * @param is_sqrt 默认开方
 */
float dis2Points(const Pose& point1, const Pose& point2,bool is_sqrt)
{
    float x = point1.x - point2.x;
    float y = point1.y - point2.y;

    if(is_sqrt)
        return sqrt(x*x +y*y);
    return x*x+y*y;
}

float disBetweenPoints(const Point& point1, const Point& point2)
{
    float x = point1.x - point2.x;
    float y = point1.y - point2.y;

    return sqrt(x*x+y*y);
}

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
                        size_t * const nearest_point_index_ptr)
{
    const auto& path_points = path.points;

    int searchDir; //搜索方向 -1:向后搜索， 1：向前搜索， 0 搜索完毕
    if(ref_point_index == 0)
    {
        ref_point_index = 1;
        searchDir = 1;
    }
    else if(ref_point_index == path_points.size()-1)
    {
        ref_point_index = path_points.size()-2;
        searchDir = -1;
    }
    else
    {
        float dis2ref  = pow(path_points[ref_point_index]->x   - x, 2) +
                pow(path_points[ref_point_index]->y   - y, 2);
        float dis2last = pow(path_points[ref_point_index-1]->x - x, 2) +
                pow(path_points[ref_point_index-1]->y - y, 2);
        float dis2next = pow(path_points[ref_point_index+1]->x - x, 2) +
                pow(path_points[ref_point_index+1]->y - y, 2);
        if(dis2next > dis2ref && dis2last > dis2ref)
            searchDir = 0;
        else if(dis2next > dis2ref && dis2ref > dis2last)
            searchDir = -1;
        else
            searchDir = 1;
    }

    //std::cout  <<  "searchDir:"  << "\t" << searchDir << "\r\n";
    while(ref_point_index>0 && ref_point_index<path_points.size()-1)
    {
        float dis2ref  = pow(path_points[ref_point_index]->x   - x, 2) +
                pow(path_points[ref_point_index]->y   - y, 2);
        float dis2last = pow(path_points[ref_point_index-1]->x - x, 2) +
                pow(path_points[ref_point_index-1]->y - y, 2);
        float dis2next = pow(path_points[ref_point_index+1]->x - x, 2) +
                pow(path_points[ref_point_index+1]->y - y, 2);
        //	std::cout  << ref_point_index << "\t" <<  sqrt(dis2last)  << "\t" << sqrt(dis2ref) << "\t" << sqrt(dis2next) << "\r\n";
        if((searchDir == 1 && dis2next > dis2ref) ||
                (searchDir ==-1 && dis2last > dis2ref) ||
                (searchDir == 0))
            break;

        ref_point_index += searchDir;
    }
    float anchor_x,anchor_y, anchor_yaw; //锚点的位置和航向
    anchor_x = path_points[ref_point_index]->x;
    anchor_y = path_points[ref_point_index]->y;
    anchor_yaw = path_points[ref_point_index]->yaw;

    if(nearest_point_index_ptr != NULL)
        *nearest_point_index_ptr = ref_point_index;
    //float dis2anchor = sqrt((x-anchor_x)*(x-anchor_x)+(y-anchor_y)*(y-anchor_y));
    //float dx = (x-anchor_x)*cos(anchor_yaw) - (y-anchor_y) * sin(anchor_yaw);
    float dy = -(x-anchor_x)*sin(anchor_yaw) + (y-anchor_y) * cos(anchor_yaw);
    return dy;
}

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
                        size_t  max_search_index)
{
    const auto& path_points = path.points;

    int searchDir; //搜索方向 -1:向后搜索， 1：向前搜索， 0 搜索完毕
    if(ref_point_index == 0)
    {
        ref_point_index = 1;
        searchDir = 1;
    }
    else if(ref_point_index == path_points.size()-1)
    {
        ref_point_index = path_points.size()-2;
        searchDir = -1;
    }
    else
    {
        float dis2ref  = pow(path_points[ref_point_index]->x   - x, 2) +
                pow(path_points[ref_point_index]->y   - y, 2);
        float dis2last = pow(path_points[ref_point_index-1]->x - x, 2) +
                pow(path_points[ref_point_index-1]->y - y, 2);
        float dis2next = pow(path_points[ref_point_index+1]->x - x, 2) +
                pow(path_points[ref_point_index+1]->y - y, 2);
        if(dis2next > dis2ref && dis2last > dis2ref)
            searchDir = 0;
        else if(dis2next > dis2ref && dis2ref > dis2last)
            searchDir = -1;
        else
            searchDir = 1;
    }

    //std::cout  <<  "searchDir:"  << "\t" << searchDir << "\r\n";
    while(ref_point_index>0 && ref_point_index<path_points.size()-1)
    {
        float dis2ref  = pow(path_points[ref_point_index]->x   - x, 2) +
                pow(path_points[ref_point_index]->y   - y, 2);
        float dis2last = pow(path_points[ref_point_index-1]->x - x, 2) +
                pow(path_points[ref_point_index-1]->y - y, 2);
        float dis2next = pow(path_points[ref_point_index+1]->x - x, 2) +
                pow(path_points[ref_point_index+1]->y - y, 2);
        //std::cout  << ref_point_index << "\t" <<  sqrt(dis2last)  << "\t" << sqrt(dis2ref) << "\t" << sqrt(dis2next) << "\r\n";
        if((searchDir == 1 && dis2next > dis2ref) ||
                (searchDir ==-1 && dis2last > dis2ref) ||
                (searchDir == 0))
            break;

        ref_point_index += searchDir;
    }
    float anchor_x,anchor_y, anchor_yaw; //锚点的位置和航向
    anchor_x = path_points[ref_point_index]->x;
    anchor_y = path_points[ref_point_index]->y;
    anchor_yaw = path_points[ref_point_index]->yaw;

    //若参考索引大于最大搜索索引,且目标物与车辆纵向距离大于一定阈值,则输出 FLT_MAX
    if(ref_point_index >= max_search_index)
    {
        anchor_x = path_points[max_search_index]->x;
        anchor_y = path_points[max_search_index]->y;
        anchor_yaw = path_points[max_search_index]->yaw;
        float dy = (x-anchor_x)*sin(anchor_yaw) + (y-anchor_y) * cos(anchor_yaw);
        //float dx = (x-anchor_x)*cos(anchor_yaw) - (y-anchor_y) * sin(anchor_yaw);
        //printf("dx:%.2f\tdy:%.2f\tref_point_index:%d\tmax_search_index:%d\n",dx,dy,ref_point_index,max_search_index);

        if(dy > 0.5)
            return FLT_MAX;
    }

    //printf("dx:%.2f\tdy:%.2f\tref_point_index:%d\n",dx,dy,ref_point_index);

    //return (x-anchor_x)*cos(anchor_yaw) - (y-anchor_y) * sin(anchor_yaw);
    return -(x-anchor_x)*sin(anchor_yaw) + (y-anchor_y) * cos(anchor_yaw);
}


/* @brief 计算点到射线的距离
 * @param ray 射线方程(x, y, yaw)
 * @param point 参考点(x, y)
 */
float dis2ray(const Pose& ray, const Point& point){
    return -(point.x-ray.x)*sin(ray.yaw) + (point.y-ray.y)*cos(ray.yaw);
}

/* @brief 计算定点停车减速度
 * @param spd_ms当前车速
 * @param remaind_dis 剩余距离
 * @param max_decel 最大制动减速度
 */
//float caculateParkingDecel(float spd_ms, float remaind_dis, float max_decel){
//    float min_decel = 0.0f5;
//    float min_valid_decel = 0.2;

//    float expect_decel = 0.0f;
//    float flag = 0.0f;

//    float margin_dis = computeYby2ends(0, 0, 10, 5.0, spd_ms);
//    if(remaind_dis - margin_dis < 0.2){
//        expect_decel = computeYby2ends(1.0, min_decel, 8.0, max_decel, spd_ms) ; //max_accel
//        flag = 1.0;
//    }else{
//        expect_decel = 0.6* (spd_ms*spd_ms) / (2 * (remaind_dis - margin_dis));
//        flag = 2.0;
//    }

//    if(expect_decel > max_decel){
//        expect_decel = max_decel;
//    }else if(expect_decel < min_decel){ // 期望制动力小于最小制动力
//        if(remaind_dis < 0.5){          // 情况1: 剩余距离较小,车辆基本处于停车, 此时保持最小制动力
//                                        // 但不应以车速较低作为判断,因为当车辆刚起步时,距离停车点很远,计算所得制动力几乎为0
//                                        // 并不能将其设置为最小制动力, 否则车辆将无法起步
//            expect_decel = min_valid_decel;
//            flag += 0.1;
//        }else{                          // 情况2: 距离停车点'极'远, 计算所得制动力'极'小
//            expect_decel = 0.0f;
//            flag += 0.2;
//        }
//    }else if(expect_decel < min_valid_decel){
//        expect_decel = min_valid_decel;
//    }

//    std::cout << flag << "  caculateStopDecel: spd_ms: " << spd_ms << "\tdis: " << remaind_dis << "\tmargin:" << margin_dis
//          << "\t expect_decel:" << expect_decel << std::endl;
//    return expect_decel;
//}

/*@brief 查找与当前点距离为dis的路径点索引
 *@param path_points 路径点集
 *@param startIndex  搜索起点索引
 *@param dis         期望距离
*/
size_t findIndexForGivenDis(const std::vector<GpsPathPoint>& path_points,
                            size_t startIndex,float dis){
    float sum_dis = 0.0f;
    for(size_t i =startIndex; i<path_points.size()-1; ++i)
    {
        sum_dis	+= disBetweenPoints(path_points[i],path_points[i+1]);
        if(sum_dis >= dis)
            return startIndex+i;
    }
    return path_points.size(); //搜索到终点扔未找到合适距离点
}

float minCurvatureInRange(const std::vector<GpsPathPoint>& path_points, size_t startIndex,size_t endIndex)
{
    float min = FLT_MAX;
    for(size_t i=startIndex; i<endIndex; i++)
    {
        if(path_points[i].curvature < min)
            min = path_points[i].curvature;
    }
    return min;
}

/*@brief local2global坐标变换
 *@param origin_x,origin_y   局部坐标系在全局坐标下的位置
 *@param theta 局部坐标系在全局坐标下的角度
 *@param local_x,local_y   点在局部坐标系下的坐标
 *@return      点在全局坐标系下的坐标
 */
std::pair<double, double> 
local2global(double origin_x,double origin_y,float theta, float local_x,float local_y)
{
    std::pair<float, float> global;
    global.first  = local_x*cos(theta) - local_y*sin(theta) + origin_x;
    global.second = local_x*sin(theta) + local_y*cos(theta) + origin_y;
    return global;
}

/*@brief global2local坐标变换
 *@param origin_x,origin_y   局部坐标系在全局坐标下的位置
 *@param theta 局部坐标系在全局坐标下的角度
 *@param local_x,local_y   点在局部坐标系下的坐标
 *@return      点在全局坐标系下的坐标
 */
std::pair<float, float> 
global2local(double origin_x,double origin_y,float theta, double global_x,double global_y)
{
    std::pair<float, float> local;
    local.first  = (global_x-origin_x)*cos(theta) + (global_y-origin_y)*sin(theta);
    local.second = -(global_x-origin_x)*sin(theta) + (global_y-origin_y)*cos(theta);
    return local;
}

/*@brief 坐标变换
 *@param origin   局部坐标系原点在全局坐标下的位姿
 *@param local   点在局部坐标系下的位置
 *@return        点在全局坐标系下的位置
 */
Point local2global(const Pose& origin, const Point& local){
    Point global;
    global.x = local.x*cos(origin.yaw) - local.y*sin(origin.yaw) + origin.x;
    global.y = local.x*sin(origin.yaw) + local.y*cos(origin.yaw) + origin.y;
    return global;
}

Pose local2global(const Pose &origin, const Pose &local){
    Pose global;
    global.x = local.x*cos(origin.yaw) - local.y*sin(origin.yaw) + origin.x;
    global.y = local.x*sin(origin.yaw) + local.y*cos(origin.yaw) + origin.y;
    global.yaw = normalizeRadAngle(origin.yaw + local.yaw);
    return global;
}

Point global2local(const Pose& origin, const Point& global)
{
    Point local;
    local.x  = (global.x-origin.x)*cos(origin.yaw) + (global.y-origin.y)*sin(origin.yaw);
    local.y = -(global.x-origin.x)*sin(origin.yaw) + (global.y-origin.y)*cos(origin.yaw);
    return local;
}

/*@brief 全局坐标系转局部坐标系
 *@param origin   局部坐标系原点在全局坐标下的位姿
 *@param global   在全局坐标系下的位姿
 *@return         在局部坐标系下的位姿
 */
Pose global2local(const Pose& origin, const Pose& global){
    Pose local;
    local.x  = (global.x-origin.x)*cos(origin.yaw) + (global.y-origin.y)*sin(origin.yaw);
    local.y = -(global.x-origin.x)*sin(origin.yaw) + (global.y-origin.y)*cos(origin.yaw);

    local.yaw = global.yaw - origin.yaw;
    if(local.yaw >= M_PI) local.yaw - 2*M_PI;
    else if(local.yaw < -M_PI) local.yaw + 2*M_PI;

    return local;
}


int sign(float val){
    if(val > 0) return 1;
    else if(val <0) return -1;
    return 0;
}

/* @brief 根据两个端点,计算特定x所对应的y值, 当x不在x1,x2范围内,取饱和值
 */
float computeYby2ends(float x1, float y1, float x2, float y2, float x){
    // y=ax+b
    float a = (y1 - y2) / (x1 - x2);
    float b = y1 - a * x1;

    if(x < x1){
        return y1;
    }else if(x < x2){
        return a * x + b;
    }else{
        return y2;
    }
}

/* @brief 判断浮点数是否为0, float精确到小数点后6为,其后的部分在内存中是不确定的
 */
bool isZeroFloat(float value){
    return fabs(value) < 10e-6;
}


bool isZeroDouble(double value){
    return fabs(value) < 10e-15;
}

std::vector<Pose> interpolate(const Pose& p1, const Pose& p2, float space){
  float distance = dis2Points(p1, p2, true);
  int n = int(distance / space + 0.5f);
  float dx = (p2.x - p1.x)/n;
  float dy = (p2.y - p1.y)/n;
  float yaw = atan2(dy, dx);

  std::vector<Pose> points(n);

  for(size_t i=0; i<n; ++i){
    Pose& point = points[i];
    point.x = p1.x + dx*i;
    point.y = p1.y + dy*i;
    point.yaw = yaw;
  }
  return points;
}

/* @brief 偏移至新路径
 * @param in_path 原路径
 * @param offset偏移量
 * @return 新路径
 */
LocalPath::Ptr offsetToNewPath(const LocalPath::Ptr in_path, float offset){
  LocalPath::Ptr new_path = std::make_shared<LocalPath>();
  new_path->vehicle_pose_index = in_path->vehicle_pose_index;
  new_path->last_safety_index = in_path->last_safety_index;
  new_path->safety_distance = in_path->safety_distance;
  new_path->start_index_in_global_path = in_path->start_index_in_global_path;
  new_path->end_index_in_global_path = in_path->end_index_in_global_path;

  new_path->frame_id = in_path->frame_id;
  new_path->resolution = in_path->resolution;
  new_path->has_curvature = in_path->has_curvature;
  new_path->final_index = in_path->final_index;

  new_path->points.resize(in_path->points.size());

  for(size_t i=0; i<in_path->points.size(); ++i){
    new_path->points[i] = posePtrOffset(in_path->points[i], offset);
  }
  return new_path;
}

geodesy::UTMPoint toUtm(double longitude, double latitude, double altitude){
    geographic_msgs::GeoPoint geo_point;
    geo_point.latitude = latitude;
    geo_point.longitude = longitude;
    geo_point.altitude = altitude;

    geodesy::UTMPoint utm;
    geodesy::fromMsg(geo_point, utm);

    return utm;
}

}//end namespace dcom



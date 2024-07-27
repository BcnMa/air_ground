#ifndef DRIVERLESS_COMMON_STRUCTS_H_
#define DRIVERLESS_COMMON_STRUCTS_H_

#include <fstream>
#include <atomic>
#include <boost/circular_buffer.hpp>
#include <boost/thread/locks.hpp>    
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>    
#include <mutex>
#include <assert.h>
#include <driverless_common/VehicleState.h>
#include <ros/ros.h>
typedef boost::shared_mutex SharedMutex;
typedef boost::unique_lock<SharedMutex> WriteLock;
typedef boost::shared_lock<SharedMutex> ReadLock;
//读写锁使用方法
//SharedMutex wr_mutex
//WriteLock wlck(wr_mutex)
//ReadLock  rlck(wr_mutex)

typedef boost::recursive_mutex RecursiveMutex;
typedef boost::recursive_mutex::scoped_lock RecursiveLock;
//递归锁,同一线程可多次获得锁, 不同线程互斥
//RecursiveMutex re_mutex
//RecursiveLock rlck(re_mutex)


/*@brief 车辆控制信息*/
typedef struct ControlCmd
{
    using Ptr = std::shared_ptr<ControlCmd>;
    using ConstPtr = std::shared_ptr<const ControlCmd>;
    ControlCmd() :
        validity(false),
        speed_validity(false),
        speed(0.0),
        roadWheelAngle(0.0),
        hand_brake(false),
        deceleration(0.0),
        acceleration(0.0),
        turnLight(0),
        stopLight(0)
    {}

    //当validity有效时,无需在对speed_validity进行判断
    //当validity无效时，需判断speed_validity是否有效!
    bool  validity;       //指令的全局有效性
    bool  speed_validity; //仅速度指令的有效性
    float speed;
    bool steer_validity;
    float roadWheelAngle;

    bool hand_brake;
    uint8_t gear;
    float deceleration;
    float acceleration;
    uint8_t turnLight; // 0 关灯,1左转,2右转
    uint8_t stopLight; // 0 关灯

    void display(const std::string& prefix){
        std::cout << prefix << "\t["
                  << "valid:" << validity << "\t"
                  << "speed:" << speed << "\t"
                  << "deceleration:" << deceleration << "\t"
                  << "angle:" << roadWheelAngle
                  << "]\r\n";
    }
} controlCmd_t;

/*@brief 停车点信息*/
class ParkingPoint{
public:
    ParkingPoint(){
        index = 0;
        parkingDuration = 0;
        parkingPointId = -100;
        isParking = false;
        isParkingOver = false;
    }
    ParkingPoint(size_t _index, float _duration){
        index = _index;
        parkingDuration = _duration;
        remainParkingDuration = _duration;
        isParking = false;
        isParkingOver = false;
    }
    ParkingPoint(size_t _index, float _duration, int32_t _id){
        index = _index;
        parkingDuration = _duration;
        parkingPointId = _id;
        remainParkingDuration = _duration;
        isParking = false;
        isParkingOver = false;
    }
    ParkingPoint(size_t _index, float _duration, double _time, bool _parking){
        index = _index;
        parkingDuration = _duration;
        parkingTime = _time;
        isParking = _parking;
    }

    // 是否刚刚完成停车
    bool isJustFinished(){
        // 若停车时间+停车周期+20s 大于当前时间, 表示停车完成不足20s
        return ros::Time::now().toSec() < parkingTime + parkingDuration + 20.0;
    }

    size_t index; //停车点在全局路径中的索引
    float  parkingDuration; //停车时长，单位s，0表示不停车,-1永久停车  //此定义不得轻易改动
    double parkingTime;     //停车时刻
    bool   isParking;       //正在停车
    bool   isParkingOver;   //停车任務完成
    float  remainParkingDuration; //剩余停车时长
    int32_t parkingPointId;  //停车点id
};
#include <thread>
class ParkingPoints{
private:
    bool sorted = false;
    int target_index=0;        // 目标停车点索引
    int last_target_index =0;  // 上次停车目标点索引
    std::mutex op_mutex;  // 操作锁  //后期修改为递归锁，防止死锁

    std::vector<ParkingPoint> points;
public:

    std::mutex& getMutex(){
      return op_mutex;
    }

    /*@brief 插入停车点, 当已存在相同位置停车点时，覆盖历史停车点
     *       !插入的停车点应确保在路径范围内，否则使用时将导致异常!
    */
    bool insert(const ParkingPoint& pp){
        std::unique_lock<std::mutex> lck(op_mutex);

        bool overwrite = false;
        for(ParkingPoint& point : points){
            if(point.index == pp.index){ // 停车点重复, 新覆盖旧
                point = pp; 
                overwrite = true;
            }
        }

        if(!overwrite){
            points.push_back(pp);

            if(sorted){ // 若插入前以及进行了排序，插入后需重新进行排序
                op_mutex.unlock();  // unlock before sort
                sort();
            }
        }

        std::cout << "insert new parking point: " << pp.index << "\t" << pp.parkingDuration << std::endl;
        return true;
    }

    void swap(std::vector<ParkingPoint>& in_points){
        std::unique_lock<std::mutex> lck(op_mutex);
        points.swap(in_points);
    }

    /* @brief 获取目标停车点
     * @param pose_index 车辆位置索引
     */
    ParkingPoint& getTargetParkingPoint(int pose_index){
        // !!Do Not  lock!!!
        assert(points.size() > 0);
        if(!sorted){
            sort();
        }

        // 若当前停车点正处于停车状态, 则直接返回当前停车点, 不根据区间进行计算
        // 因为当车辆处于停车状态时, 可能已经超过停车点, 进入到下一个区间
        if(points[target_index].isParking){
            return points[target_index];
        }
        
        if(points[last_target_index].isParkingOver){
            return points[target_index];
        }

        // 0----pt1----pt2----pt3----pt4----0

        // 当前索引大于最后一个停车点索引, 或者小于第一个停车点, 将第一个停车点置为目标点
        if(pose_index > points.back().index || pose_index <= points[0].index){
            if(target_index != 0){
                last_target_index = target_index;
                target_index = 0;
            }
        }else{
            // 查找索引所在区间
            for(int i=target_index; i<points.size()-1; ++i){
                if(pose_index > points[i].index && pose_index <= points[i+1].index){
                    if(target_index != i + 1){
                        last_target_index = target_index;
                        target_index = i + 1;
                    }
                    break;
                }
            }
        }
        return points[target_index];
    }

    // 切换到下一停车点
    void next(){
        // !!Do Not  lock!!!

        last_target_index = target_index;
        // 考虑环形路径
        target_index = (target_index+1)%points.size();
        points[target_index].isParkingOver = false;
    }

    size_t size() const {return points.size();}
    // void push_back(const ParkingPoint& point){
    //     points.push_back(point);
    // }

    const ParkingPoint& operator[](size_t i)const  {return points[i];}
    // ParkingPoint& operator[](size_t i)             {return points[i];}

    void sort() {//停车点由小到大排序
        std::cerr << "start sort... " << points.size() << std::this_thread::get_id() << std::endl;
        std::unique_lock<std::mutex> lck(op_mutex);
        std::cerr << "sort lock ok" << std::this_thread::get_id() << std::endl;
        std::sort(points.begin(),points.end(),
                  [](const ParkingPoint& point1,const ParkingPoint& point2){
            return point1.index < point2.index;
        });
        sorted = true;
        
        // 重新排序后复位索引
        target_index = 0;        // 目标停车点索引
        last_target_index = 0;  // 上次停车目标点索引
        std::cerr << "ok sort..." << std::endl;
    }

    bool isSorted() const {return sorted;}

    void print(const std::string& prefix) const{
        for(auto &point:points)
            printf("[%s] parking point index: %lu  duration: %.1f  id: %d\r\n",
                   prefix.c_str(), point.index,point.parkingDuration, point.parkingPointId);
    }

    void clear() {
        points.clear();
        sorted = false;
        target_index =0;
        last_target_index = 0;
    }

};

/*@brief 交通灯点信息*/
class TrafficLightPoint
{
public:
    TrafficLightPoint()
    {
        index = 0;
        parkingDuration = 0;
        isParking = false;
    }
    TrafficLightPoint(size_t _index, float _duration)
    {
        index = _index;
        parkingDuration = _duration;
        isParking = false;
    }
    TrafficLightPoint(size_t _index, float _duration, double _time, bool _parking)
    {
        index = _index;
        parkingDuration = _duration;
        parkingTime = _time;
        isParking = _parking;
    }

    size_t index; //交通灯点在全局路径中的索引
    float  parkingDuration; //停车时长，单位s，0表示不停车,-1永久停车  //此定义不得轻易改动
    double parkingTime;     //停车时刻
    bool   isParking;       //正在停车
};

class TrafficLightPoints
{
public:
    std::vector<TrafficLightPoint> points;
    size_t next_index = 0;
    bool sorted = false;

    size_t size() const {return points.size();}
    void push_back(const TrafficLightPoint& point)
    {
        points.push_back(point);
    }
    const TrafficLightPoint& operator[](size_t i)const  {return points[i];}
    TrafficLightPoint& operator[](size_t i)             {return points[i];}

    bool available() const { return next_index  < points.size();}

    void sort() //按索引由小到大排序
    {
        std::sort(points.begin(),points.end(),
                  [](const TrafficLightPoint& point1,const TrafficLightPoint& point2)
        {return point1.index < point2.index;});
        sorted = true;
    }

    bool isSorted() const {return sorted;}

    void print(const std::string& prefix) const
    {
        for(auto &point:points)
            printf("[%s] traffic light point index: %lu  duration: %.1f",prefix.c_str(), point.index,point.parkingDuration);
    }

    TrafficLightPoint& next()
    {
        if(!available())
            next_index = 0;

        return points[next_index];
    }
public:

    void clear()
    {
        points.clear();
        next_index = 0;
        sorted = false;
    }

};

/*@brief 路径转向区间信息 */
class TurnRange
{
public:
    enum TurnType
    {
        TurnType_Left = -1,
        TurnType_None = 0,
        TurnType_Right = 1,
    };

    int type;
    size_t start_index;
    size_t end_index;

    TurnRange(int _type, size_t _start_index, size_t _end_index)
    {
        type = _type;
        start_index = _start_index;
        end_index = _end_index;
    }
    uint8_t getCurrentLight() const
    {
        if(type == TurnType_Left) // 0 关灯,1左转,2右转
            return 1;
        else if(type == TurnType_Right)
            return 2;
        else
            return 0;
    }
};

class TurnRanges
{
public:
    std::vector<TurnRange> ranges;

    size_t size() const {return ranges.size();}
    void clear()
    {
        ranges.clear();
    }

};

/*@brief 路径限速区间信息*/
class SpeedRange
{
public:
    float speed;
    size_t start_index;
    size_t end_index;

    SpeedRange(float _speed, size_t _start_index, size_t _end_index)
    {
        speed = _speed;
        start_index = _start_index;
        end_index = _end_index;
    }
};

class SpeedRanges
{
public:
    std::vector<SpeedRange> ranges;

    size_t size() const {return ranges.size();}
    void clear()
    {
        ranges.clear();
    }
};

/*@brief 站点信息*/
class StationPoint{
public:
    std::string name; //站点名称
    size_t index; //站点索引
    bool is_stay; //是否停留
    int stay_duration; //停留时间s
    int station_id; //站点id
    
    StationPoint( const std::string& _name, size_t _idx,
                 bool _is_stay=true, int _stay_duration=30){
        name = _name;
        index = _idx;
        is_stay = _is_stay;
        stay_duration = _stay_duration;
    }
    
    StationPoint(int _sid, const std::string& _name, size_t _idx,
                 bool _is_stay=true, int _stay_duration=30){
        station_id = _sid;
        name = _name;
        index = _idx;
        is_stay = _is_stay;
        stay_duration = _stay_duration;
    }
};

class StationPoints{
public:
    std::vector<StationPoint> points;
    bool sorted = false;

    size_t size() const {return points.size();}
    void clear()
    {
        points.clear();
    }

    void sort() //由小到大排序
    {
        std::sort(points.begin(),points.end(),
                  [](const StationPoint& point1,const StationPoint& point2)
        {return point1.index < point2.index;});
        sorted = true;
    }

    void print(const std::string& prefix) const
    {
        for(auto &point:points)
            printf("[%s] station point index: %lu  name: %s  stay_time: %d\n",
                   prefix.c_str(), point.index, point.name.c_str(), point.is_stay?point.stay_duration:0);
    }
};

/*@brief 位置信息*/
class Point
{
public:
    double x,y,z;
    Point(){}
    Point(double _x, double _y, double _z=0.0):
        x(_x), y(_y), z(_z){}

    float disTo(const Point& point, bool _sqrt=true) const{
        float dx = point.x - x;
        float dy = point.y - y;
        if(_sqrt){
            return sqrt(dx*dx + dy*dy);
        }else{
            return dx*dx + dy*dy;
        }
    }

    float disTo(Point* const point, bool _sqrt=true) const{
        return disTo(*point, _sqrt);
        //    float dx = point->x - x;
        //    float dy = point->y - y;
        //    if(_sqrt){
        //      return sqrt(dx*dx + dy*dy);
        //    }else{
        //      return dx*dx + dy*dy;
        //    }
    }
};

/*@brief 位姿信息*/
class Pose : public Point
{
public:
//    Pose(){}
//    Pose(double _x, double _y, double _z, double _yaw):

    double yaw;
};

class PathPoint : public Pose{
public:
    using Ptr = std::shared_ptr<PathPoint>;
    using ConstPtr = std::shared_ptr<const PathPoint>;

    float curvature = 0.0;

    PathPoint(){}
    PathPoint(const Pose& pose){
        //        *((Pose *)this) = pose;
        this->x = pose.x;
        this->y = pose.y;
        this->yaw = pose.yaw;
    }    
};


/*@brief frenet坐标系路径点信息*/
class FrenetPathPoint : public Pose
{
public:
    float curvature;
    float left_width;
    float right_width;
    float frenet_s;
    float frenet_d;
};

/*@brief 路径点信息*/
class GpsPathPoint : public PathPoint
{
public:
    using Ptr = std::shared_ptr<GpsPathPoint>;
    using ConstPtr = std::shared_ptr<const GpsPathPoint>;

    double longitude;
    double latitude;
    float left_width;
    float right_width;
};

/* @brief 路径基类 模板类，可根据路径点类型设置全局路径和局部路径
 */
template <typename PointT>
class Path{
public:
    typedef std::shared_ptr<PointT> PointPtrT;
    typedef std::shared_ptr<const PointT> PointConstPtrT;

    std::vector<PointPtrT> points;
    std::vector<float> offsets;      

    std::string frame_id;
    float resolution;
    bool  has_curvature=false;               //是否包含路径曲率

    //被延伸的路径最后一个点与路径终点不一致
    //终点索引，路径被载入时的最后一个点的索引(global/local)
    int final_index = 0;

    void clear(){
        points.clear();
        offsets.clear();
        resolution = 0.0;
        has_curvature = false;
        final_index = 0;
    }

    size_t size() const {return points.size();}
    const PointConstPtrT& operator[](size_t i) const {
        assert(i < points.size());
        return points[i];
    }
    PointPtrT& operator[](size_t i) {
        assert(i < points.size());
        return points[i];
    }
    const PointT& at(size_t i) const {
    	if(i >= points.size()){
    		std::cerr << "const PointT& at(size_t i) const" << std::endl;
        	assert(i < points.size());
        }
        return *points[i];
    }
    PointT& at(size_t i) {
        if(i >= points.size()){
    		std::cerr << "PointT& at(size_t i)" << std::endl;
        	assert(i < points.size());
        }
        return *points[i];
    }

    size_t toIndex(size_t i){
        return i%points.size();
    }
};

typedef PathPoint LocalPathPoint;
typedef GpsPathPoint GlobalPathPoint;

/* @brief 局部路径
 */
class LocalPath : public Path<LocalPathPoint>{
public:
    using Ptr = std::shared_ptr<LocalPath>;
    using ConstPtr = std::shared_ptr<const LocalPath>;

    size_t vehicle_pose_index; //汽车位置最近点索引(在当前局部路径中的索引)
    size_t last_safety_index; //最后一个安全索引, 之后的路径点(如果有)为非安全路径, 仅作为轨迹参考
    float safety_distance;    //last_safety_index对应的距离

    size_t start_index_in_global_path; //当前局部路径的起点在全局路径点中的索引
    size_t end_index_in_global_path;   //终点在全局路径点中的索引
    double update_time; // 更新时间
};

/* @brief 全局路径
 */
class GlobalPath : public Path<GlobalPathPoint>{
public:
    using Ptr = std::shared_ptr<GlobalPath>;
    using ConstPtr = std::shared_ptr<const GlobalPath>;

    int pose_index=0;                 // 距离车辆当前位置最近的路径点索引

    ParkingPoints park_points;         //停车点信息
    TurnRanges    turn_ranges;		   //转向区间信息
    SpeedRanges   speed_ranges;        //限速区间信息
    StationPoints station_points;      //站点信息

private:
    bool is_circular = false;          //是否为循环路径
    bool is_extended = false;         //是否被延伸
    std::vector<float> distances;     //每个路径点相对于起点的路程

public:
    GlobalPath(){} //当定义了拷贝构造函数时，编译器将不提供默认构造函数，需显式定义

    void clear(){     //清空路径信息
        Path<GlobalPathPoint>::clear();

        pose_index = 0;
        park_points.clear();
        turn_ranges.clear();
        speed_ranges.clear();
        station_points.clear();
        is_circular = false;
        is_extended = false;
        distances.clear();
    }
    
    ParkingPoint getCurParkingStation() const{
        ParkingPoint pt(0,100,-100);
        for(int i = 0; i < park_points.size(); i++){
            if((this->pose_index >= park_points[i].index - 10) && (this->pose_index <= park_points[i].index + 10)){
                pt = park_points[i];
                break;
            }
        }
        return pt; //没有到达站点
    }
    
    bool setCirularEnable(){
        if(points.size() == 0){
            std::cerr << "Please load path points before setCirularEnable" << std::endl;
            return false;
        }

        if(is_circular){
            return is_circular;
        }
        // 若路径已被延伸, 则清除被延伸部分
        if(is_extended){
            points.resize(final_index+1);
        }
        if(!generateCyclePath()){
            return is_circular;
        }
        is_circular = true;
        return is_circular;
    }

    bool isCirular() const{
        return is_circular;
    }

    float frenetS(int index){
        int dis_len = distances.size();
        if(dis_len == 0){
            calculateDistances();
            dis_len = distances.size();
        }
        if(is_circular){
            // 如果索引大于总数,则表示为新的一圈
            if(index >= dis_len){
                int n_turns = index/dis_len;
                int idx = index % dis_len;

                return distances[idx] + n_turns * distances.back();
            }
            if(index < 0){ // 索引为负，距离为负
                int n_turns = (-index)/dis_len;
                int idx = dis_len * (n_turns+1) + index;
                
                return -n_turns * distances.back() +  (distances[idx] - distances.back());
            }
        }else{
            if(index >= dis_len || index < 0){
                std::cerr << "abnormal index: " << index << std::endl;
                assert(index < dis_len && index >= 0);
            }
        }
        return distances[index];
    }
    /* @brief 计算两点区间的路程
   * @param start 起点索引
   * @param end 终点索引
   */
    float rangeDistance(size_t start, size_t end){
        if(distances.size() == 0){
            calculateDistances();
        }

        // 1. 循环路径需要对索引进行取模,防止越界
        // 2. 当start大于end时, 其路程为 start->路径终点->路径起点->end
        if(is_circular){
            if(end < start || end >= distances.size()){
                return distances[end%distances.size()]
                        - distances[start%distances.size()] + distances.back();
            }
        }else{
            if(start >= distances.size() || end >= distances.size()){
                std::cerr << "distances.size(): " << distances.size() << "\t"
                          << "start: " << start << "\t" << "end: " << end << std::endl;
                assert(start < distances.size() && end < distances.size());
            }
        }
        return distances[end] - distances[start];
    }

#if 0
    /* @brief 将当前路径处理为环形路径
   *  1. 首先点判断是否重叠,若重叠先将重叠的去除
   *  2. 根据处理后的起始点和终止点进行线性插值
   */
    //环形路径生成
    //首先点判断是否重叠,若重叠先将重叠的去除
    //根据处理后的起始点和终止点进行线性插值
    bool generateCyclePath(){
        int end_index = points.size()-1;
        std::cout << "[raw_num_points]:  " << end_index + 1 << std::endl;
        int start_index = 0;

        int min_dis = 10000;
        for(int i=end_index, j=0; j<100; i--,j++){
            float dy = (points[i]->y - points[start_index]->y);
            float dx = (points[i]->x - points[start_index]->x);
            float dis = sqrt(dy*dy + dx*dx);
            if(dis < min_dis){
                end_index = i;
                min_dis = dis;
            }
        }

        //获取最近点到最后一个点的个数
        //多删除一些点保证连接顺利
        int delete_nums = (points.size()-end_index) * 2 + 10;
        int start_delete_index = points.size() - delete_nums;
        points.erase(points.begin()+start_delete_index, points.end());

        //获取删除点后的最终点的索引
        end_index = points.size()-1;
        std::cout << "[aftdel_num_points]:  " << end_index + 1 << std::endl;

        float dy = (points[end_index]->y - points[start_index]->y);
        float dx = (points[end_index]->x - points[start_index]->x);
        int sign_x, sign_y;
        float k = dy / dx;
        float theta = tan(k);
        float raw_dis = sqrt(dy*dy + dx*dx);

        if(dy >= 0)
            sign_y = -1;
        else
            sign_y = 1;
        if(dx >= 0)
            sign_x = -1;
        else
            sign_x = 1;

        float delta_dis = 0.1;
        float remaind_dis = 0.0;
        for(size_t i=1;;++i){
            GlobalPathPoint::Ptr point(new GlobalPathPoint);
            point->x = points[end_index]->x + delta_dis*cos(theta)*i*sign_x;
            point->y = points[end_index]->y + delta_dis*sin(theta)*i*sign_y;
            point->curvature = theta;
            points.push_back(point);
            remaind_dis += delta_dis;
            //减去0.1保证线性插值最后不会再次重叠
            if(remaind_dis > raw_dis - 0.1)
                break;
        }
        return true;
    }

#endif

    //路径是否可循环驾驶
    //通过路径起点和尾点的距离是否在一定范围内，判断路径是否可循环驾驶
    bool recyclable(float max_dis = 1.0) const{
        //终点索引大于点的个数，程序异常！
        //终点索引过小则路径较短，无法‘循环’驾驶
        if(final_index >= points.size() || final_index < 2)
            return false;

        //起点与终点距离过远，无法循环驾驶
        float dis = points[0]->disTo(points[final_index].get());
        if(dis > max_dis){
            std::cout << "[check only]: path not recyclable. start->end: " << dis << "m!" << std::endl;
            return false;
        }
        return true;
    }

    //获取路径上距离参考点(ref_p)最近的点
    GlobalPathPoint nearest(const Point& ref_p, size_t *index=nullptr) const{
        float min_dis = 99999;
        size_t nearest_index = 0;
        for(size_t i=0; i<points.size(); ++i){
            float dis = ref_p.disTo(*points[i]);
            //std::cout << i << "\t" << dis << std::endl;
            if(dis < min_dis){
                min_dis = dis;
                nearest_index = i;
            }
        }
        if(index){
            *index = nearest_index;
        }
        //std::cout << "nearest_index:" << nearest_index <<"\tmin_dis:" << min_dis << std::endl;
        return *points[nearest_index];
    }

    /* @brief 在指定索引范围内查找point的最近点索引
   * @param point 目标点
   * @param ref_index 参考索引
   * @param range 搜索范围[ref_index-range, ref_index+range]
   */
    int nearestIndex(const Point& point, int ref_index, int range){
        float min_dis2 = FLT_MAX;
        int nearest_index = ref_index;
        int pts_size = points.size();
        if(is_circular){ // 循环路径需要考虑搜索范围跨界问题
            int start = (ref_index-range+pts_size)%pts_size;
            int end = (ref_index+range)%pts_size;
            if(end < start){
                end += pts_size;
            }

            for(int i=start; i<end; ++i){
                int index = i%pts_size;
                float dis2 = point.disTo(*points.at(index), false) ;
                if(dis2 < min_dis2){
                    nearest_index = index;
                    min_dis2 = dis2;
                }
            }
        }else{
            int start = std::max(0, pose_index-range);
            int end = std::min(pts_size, pose_index+range);
            for(int index=start; index<end; ++index){
                float dis2 = point.disTo(*points.at(index), false) ;
                if(dis2 < min_dis2){
                    nearest_index = index;
                    min_dis2 = dis2;
                }
            }
        }
        return nearest_index;
    }

    /* @brief 给定距离查找路径点索引值
     * @param start 起点索引
     * @param dis 目标距离
     * @param increment 索引搜索增量, 增量越大,搜索越快,但精度越低
     */
    // int getIndexByGivenDistance(int start, float dis, int increment=5){
    //     int pts_size = points.size();

    //     float start_frenetS = frenetS(start);
    //     int res_index = start;
    //     while(true){
    //         res_index+=increment;

    //         // 循环路径一直查找,直到距离满足, 非循环路径则需要进行越界约束
    //         if(!is_circular && res_index >= pts_size){
    //             res_index = pts_size-1;
    //             break;
    //         }

    //         if(frenetS(res_index) - start_frenetS > dis){
    //             break;
    //         }
    //     }
    //     return res_index % pts_size;
    // }


    int getIndexByGivenDistance(int start, float dis, int increment=5){
        int pts_size = points.size();

        float start_frenetS = frenetS(start);
        int res_index = start;
        while(true){
            res_index += increment;
            // 循环路径一直查找, 直到距离满足, 非循环路径则需要进行越界约束
            
            if(increment > 0){
                // 循环路径一直查找, 直到距离满足, 非循环路径则需要进行越界约束
                if(!is_circular && res_index >= pts_size){
                    res_index = pts_size-1;
                    break;
                }
           
                if(frenetS(res_index) - start_frenetS > dis){
                    break;
                }
            }else if(increment < 0){
                if(!is_circular && res_index < 0){
                    res_index = 0;
                    break;
                }
                
                if(start_frenetS - frenetS(res_index) > dis){
                    break;
                }
            }
            // std::cout << "getIndexByGivenDistance: check index : " << res_index << std::endl;
            // std::cout << "getIndexByGivenDistance distance: " << (frenetS(res_index) - start_frenetS) << std::endl;
        }
        // std::cout << "getIndexByGivenDistance index / distance: " << res_index << "\t" 
        //           << (frenetS(res_index) - start_frenetS) << std::endl;

        return res_index % pts_size;
    }

    /* @brief 给定范围内路径点的个数
     * @param start 起点索引
     * @param end 终点索引
     * @return 点数
     */
    int pointsSizeInRange(int start, int end){
        if(end >= start){
            return end - start + 1;
        }

        if(is_circular){
            while (end < start) {
                end += points.size();
            }
            return end - start + 1;
        }else{
            return start - end + 1;
        }
    }
    /* @brief 拷贝指定范围内的点
     * @param start 起点索引
     * @param end 终点索引
     * @param pts 目标点集
     */
    template <typename PointT>
    void copyPointsInRange(int start, int end, std::vector<PointT>& pts){
        pts.clear();
        pts.reserve(pointsSizeInRange(start, end));
        if(end >= start){
            for(int i=start; i<=end; ++i){
                pts.push_back(points[i]);
            }
            return ;
        }
        int pts_size = points.size();
        if(is_circular){
            while (end < start) {
                end += points.size();
            }
            for(int i=start; i<=end; ++i){
                pts.push_back(points[i%pts_size]);
            }
        }else{
            assert(start < end);
        }
    }

    bool calculateDistances(){
        if(points.size() == 0)
            return false;
        size_t cnt = points.size();
        distances.resize(cnt);
        distances[0] = 0;
        for(size_t i=1; i<cnt; ++i){
            float dx = (points[i]->x - points[i-1]->x);
            float dy = (points[i]->y - points[i-1]->y);
            distances[i] = distances[i-1] + sqrt(dx*dx+dy*dy);
            // std::cout << "distances[i]: " << distances[i] << std::endl;

        }

        return true;
    }

    bool finish() const {
        // 循环路径无finish
        if(!is_circular){
            return pose_index>=final_index;
        }
        return false;
    }

    float remaindDis(){
        float dis = (final_index - pose_index) * resolution;
        if(dis < 0) dis = 0;
        return dis;
    }

    //延伸路径, 在路径的末尾增加一段距离的点
    bool extend(float length){
        if(is_extended){
            return false;
        }

        //取最后一个点与倒数第n个点的连线向后插值
        //总路径点不足n个,退出
        int n = 5;
        //std::cout << "extendPath: " << points.size() << "\t" << points.size()-1 << std::endl;
        if(points.size()-1 < n){
            printf("path points is too few (%lu), extend path failed",points.size()-1);
            return false;
        }
        final_index = points.size()-1;
        int endIndex = points.size()-1;

        float dx = (points[endIndex]->x - points[endIndex-n]->x)/n;
        float dy = (points[endIndex]->y - points[endIndex-n]->y)/n;
        float ds = sqrt(dx*dx+dy*dy);

        float remaind_dis = 0.0;
        for(size_t i=1;;++i){
            GlobalPathPoint::Ptr point(new GlobalPathPoint);
            point->x = points[endIndex]->x + dx*i;
            point->y = points[endIndex]->y + dy*i;
            point->curvature = 0.0;
            points.push_back(point);
            remaind_dis += ds;
            if(remaind_dis > length)
                break;
        }
        is_extended = true;
        return true;
    }

    /* @brief 获取路段宽度
     * @param left 左侧宽度
     * @param right 右侧宽度
     * @param index 指定的索引点
     * @param index_valid 索引点是否有效，索引点无效时则查询当前位置的路段宽度
     */
    void getSectionWidth(float& left, float& right, size_t index=0, bool index_valid=false) const{
      if(!index_valid){
        index = this->pose_index;
      }
      left = points[index]->left_width;
      right = points[index]->right_width;
    }

private:
    bool generateCyclePath(){
        if(points.size() == 0){
            return false;
        }

        if(points.size() < 10){ //路径点过少
            return false;
        }

        int end_index = points.size()-1;
        int start_index = 0;

        float min_dis = FLT_MAX;
        for(int i=points.size()-1; i>points.size()/3; --i){
            float dy = (points[i]->y - points[start_index]->y);
            float dx = (points[i]->x - points[start_index]->x);
            float dis = sqrt(dy*dy + dx*dx);
            if(dis < min_dis){
                end_index = i;
                min_dis = dis;
            }
        }

        if(min_dis > 10.0){ // 距离过远, 无法闭环
            return false;
        }else if(min_dis > 0.2){ // 距离较远, 插值补充
            // points.resize(end_index+1); // 清空重合点
            points.resize(end_index); // 清空重合点, 包括最后一个重合点
            // 开始插值
            GlobalPathPoint::Ptr p_s = points.back();
            GlobalPathPoint::Ptr p_e = points.front();

        }else{
            points.resize(end_index);
        }

        final_index = points.size() - 1;

        return true;
    }
};

/*@brief 车辆参数 */
class VehicleParams{
public:
    float max_roadwheel_angle;
    float min_roadwheel_angle;
    float min_steering_radius;  //最小转向半径 m
    float wheel_base;   //轴距 m
    float wheel_track;  //轮距 m
    float width;        //车宽 m
    float length;       //车长 m

    float max_deceleration;     // m/s2
    float max_acceleration;     // m/s2
    float max_side_acceleration;// m/s2 最大侧向加速度
    float speed_control_period; // s
    float steer_control_period; // s
    float max_speed;            // km/h
    float min_speed;            // km/h
    float max_steering_speed;   // deg/s
    float steer_clearance;      //转向间隙 deg

    bool validity;
    VehicleParams(){
        validity = false;
    }
};

/*@brief 车辆状态 内部状态+外部状态
 * 更新车辆状态的线程利用类方法进行更新
 * 读取车辆状态的线程先创建副本，然后直接访问副本成员
*/
#define LOCK true
#define UNLOCK  false
//此类中的读写锁是否具有意义，如果在外部全局加锁，是否会更合理
class VehicleState{
public:
    using Ptr = std::shared_ptr<VehicleState>;
    using ConstPtr = std::shared_ptr<const VehicleState>;

    bool base_ready = true; //线控系统就绪(硬件急停释放, 自动驾驶开关闭合等要素)
    bool driverless_mode;   //是否为自动驾驶模式
    uint8_t gear;           //档位
    float   speed = 0.0;        //车速 km/h
    float   acceleration = 0.0;
    float   road_wheel_angle = 0.0;  //前轮转角
    Pose    pose;         //车辆位置

    bool speed_validity = false;
    bool steer_validity = false;
    bool pose_validity  = false;

    SharedMutex wr_mutex;//读写锁

public:
    void setSpeed(const float& val)	{
        WriteLock writeLock(wr_mutex);
        speed = val;
    }

    void setSteerAngle(const float& val){
        WriteLock writeLock(wr_mutex);
        road_wheel_angle = val;
    }

    void setPose(const Pose& val){
        WriteLock writeLock(wr_mutex);
        pose = val;
    }

    void setGear(uint8_t g)	{
        WriteLock lc(wr_mutex);
        gear = g;
    }

    void setPoseValid(bool flag){
        WriteLock writeLock(wr_mutex);
        pose_validity = flag;
    }

    uint8_t getGear(){
        ReadLock readLock(wr_mutex);
        return gear;
    }

    float getSpeed(bool lock = UNLOCK){
        if(lock)
        {
            ReadLock readLock(wr_mutex);
            return speed;
        }
        return speed;
    }
    float getSteerAngle(bool lock = UNLOCK){
        if(lock)
        {
            ReadLock readLock(wr_mutex);
            return road_wheel_angle;
        }
        return road_wheel_angle;
    }

    Pose getPose(bool lock = UNLOCK){
        if(lock)
        {
            ReadLock readLock(wr_mutex);
            return pose;
        }
        return pose;
    }

    bool getPoseValid() const{
        return pose_validity;
    }

    // 车速是否足够小, 用于停车判断
    bool speedLowEnough() const{
        return fabs(speed) < 0.1;
    }

    bool isDriveGear() const{
        return gear == driverless_common::VehicleState::GEAR_DRIVE;
    }

    bool isReverseGear() const{
        return gear == driverless_common::VehicleState::GEAR_REVERSE;
    }

    bool isNeutralGear()const {
        return gear == driverless_common::VehicleState::GEAR_NEUTRAL;
    }

    VehicleState(){} //当定义了拷贝构造函数时，编译器将不提供默认构造函数，需显式定义

    // 重载拷贝构造和赋值构造函数, 以避免读写锁拷贝报错
    VehicleState(const VehicleState& obj){
        WriteLock lck(wr_mutex);
        this->driverless_mode = obj.driverless_mode;
        this->base_ready  = obj.base_ready;
        this->gear        = obj.gear;
        this->speed       = obj.speed;
        this->road_wheel_angle = obj.road_wheel_angle;
        this->pose        = obj.pose;
        this->speed_validity    = obj.speed_validity;
        this->steer_validity    = obj.steer_validity;
        this->pose_validity     = obj.pose_validity;
    }
    const VehicleState& operator=(const VehicleState& obj){
        WriteLock lck(wr_mutex);
        this->driverless_mode = obj.driverless_mode;
        this->base_ready  = obj.base_ready;
        this->gear        = obj.gear;
        this->speed       = obj.speed;
        this->road_wheel_angle = obj.road_wheel_angle;
        this->pose        = obj.pose;
        this->speed_validity    = obj.speed_validity;
        this->steer_validity    = obj.steer_validity;
        this->pose_validity     = obj.pose_validity;
        return *this;
    }

    bool validity(std::string& info) const{
        bool ok = true;
        if(!speed_validity){
            info += "[speed] ";
            ok = false;
        }
        if(!steer_validity){
            info += "[steer] ";
            ok = false;
        }
        if(!pose_validity){ //the pose from gps is invailed!
            info += "[pose] ";
            ok = false;
        }
        if(!base_ready){
            info += "[base] ";
            ok = false;
        }

        if(!ok)
            info = "Invalid " + info;

        return ok;
    }


};

/* @brief 加速度计算器
 *
 */
class AccelCalculator{
    struct StampedSpeed{
        using Ptr = std::shared_ptr<StampedSpeed>;
        StampedSpeed(double _sped, double _time): speed(_sped),time(_time){}
        float speed;
        double time;
    };
public:
    AccelCalculator(float delta_size=5):stamped_speed_buffer_(delta_size){}
    float update(float spd_kph){
        stamped_speed_buffer_.push_back(StampedSpeed::Ptr(new StampedSpeed (spd_kph/3.6, ros::Time::now().toSec())));
        if(stamped_speed_buffer_.full()){
            const StampedSpeed::Ptr& stamp_speed1 = stamped_speed_buffer_.front();
            const StampedSpeed::Ptr& stamp_speed2 = stamped_speed_buffer_.back();
            float dv = stamp_speed2->speed - stamp_speed1->speed;
            float dt = stamp_speed2->time - stamp_speed1->time;
            return dv/dt;
        }else{
            return 0.0;
        }
    }
    void reset(){
        stamped_speed_buffer_.clear();
    }

private:
    boost::circular_buffer<StampedSpeed::Ptr> stamped_speed_buffer_; //包含时间戳的速度列表,用于计算车辆加速度
};

/* @brief 调试信息打印
 */
class Debug{
public:
    Debug(const std::string& _header, bool enable=true):
        header_(_header),
        enable_(enable),
        of_null_("/dev/null"){
    }

    void setEnable(bool enable){
        enable_ = enable;
    }

    std::ostream& operator()(){
        if(!enable_){
            return of_null_;
        }
        return std::cout << "[" << header_ << "] ";
    }

    std::ostream& operator()(int percision){
        if(!enable_){
            return of_null_;
        }
        return std::cout << "[" << header_ << "] " << std::setprecision(percision);
    }

private:
    std::string header_;
    std::ofstream of_null_;
    bool enable_;
};

#endif

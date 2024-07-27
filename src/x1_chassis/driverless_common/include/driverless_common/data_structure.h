#ifndef DATA_STRUCTURE_H
#define DATA_STRUCTURE_H
#include <vector>
#include <iostream>
#include <memory>
#include <mutex>
#include <assert.h>

namespace dcom {

/*@brief 自定义容器，存储大量数据，避免vector扩容数据拷贝问题
 * 将数据以批次pitch的方式存储,当前picth存满后,创建一个新的pitch进行数据存储
 * 当批数量达到最大时,将删除一部分以便于继续存储
 * VectorArray<int> datas(500, 500);  每个picth可存储500个数据, 最多存储500个pitch
 * datas.push_back(10);
 *
 */
template<typename T>
class VectorArray{
  typedef std::vector<T> Pitch;
  typedef std::shared_ptr<Pitch> SharedPtrPitch;
  typedef std::vector<SharedPtrPitch> Pitchs;
public:
    VectorArray(size_t _pitch_capicity=500, size_t _max_picth_cnt=500):
        pitch_capicity(_pitch_capicity),
        max_picth_cnt(_max_picth_cnt)
    {
        pitchs.reserve(max_picth_cnt);  // 预留空间
        this->add_patch(); // 创建第一个批,开始存放数据
    }

    // 2022.04.01修正bug
    // 先判断当前pitch是否已满, 已满则新增pitch,然后将当前数据放入新pitch
    // 不可先新增数据再判断是否以满,否则将导致新pitch没有数据,调用back时内存出错
    void push_back(const T& data){
        std::lock_guard<std::mutex> lck(mutex);
        if(now_pitch->size() == now_pitch->capacity()){ //当前批容量已满，移动批指针
            this->add_patch();
        }

        now_pitch->push_back(data); //数据添加到在当前批中
    }

    template <typename... Args>
    void emplace_back(Args&&... args){
        std::lock_guard<std::mutex> lck(mutex);
        if(now_pitch->size() == now_pitch->capacity()){ //当前批容量已满，创建新批
            this->add_patch();
        }

        now_pitch->emplace_back(args...); //数据添加到在当前批中
    }

    const T& operator[](size_t i) const{
        assert(i < size());
        size_t pitch_idx = i / pitch_capicity;
        size_t data_idx = i % pitch_capicity;

        return pitchs[pitch_idx]->at(data_idx);
    }

    T& operator[](size_t i){
        std::lock_guard<std::mutex> lck(mutex);
        assert(i < size());
        size_t pitch_idx = i / pitch_capicity;
        size_t data_idx = i % pitch_capicity;

        return pitchs[pitch_idx]->at(data_idx);
    }

    T& front(){
        std::lock_guard<std::mutex> lck(mutex);
        return pitchs[0]->front();
    }

    T& back(){
        std::lock_guard<std::mutex> lck(mutex);
        return now_pitch->back();
    }

    const T& front()const {
        return pitchs[0]->front();
    }

    const T& back() const{
        return now_pitch->back();
    }

    size_t size() const{
        return now_pitch->size() + (pitchs.size()-1) * pitch_capicity;
    }

    void clear(){
        std::lock_guard<std::mutex> lck(mutex);

        // 删除所有数据
        for(int i=0; i<pitchs.size(); ++i)
            pitchs[i]->clear();
        pitchs.clear();
        this->add_patch(); // 创建第一个批, 准备继续存放数据
    }

private:
    // 创建新批次容器
    void add_patch(){
        if(pitchs.size() == max_picth_cnt){
            // 批数量达到最大, 删除一定数量的批, 若只删除一个, 删除频率较高, 因此每次删除一部分
            pitchs.erase(pitchs.begin(), pitchs.begin()+max_picth_cnt/3);
        }
        now_pitch = SharedPtrPitch(new Pitch);
        now_pitch->reserve(pitch_capicity);
        pitchs.push_back(now_pitch);
    }

private:
    const size_t pitch_capicity;    //每批的容量
    const size_t max_picth_cnt;     //最大批个数

    SharedPtrPitch now_pitch=nullptr; //当前批指针

    Pitchs pitchs;
    std::mutex mutex;
};

} // end dcom


#endif // DATA_STRUCTURE_H

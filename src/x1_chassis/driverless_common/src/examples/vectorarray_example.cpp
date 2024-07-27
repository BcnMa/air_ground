#include <driverless_common/structs.h>
#include <driverless_common/exlcm/CustomMsg.hpp>
#include <driverless_common/data_structure.h>


class Data{
public:
  Data(int a, float b){
    A = a;
    B = b;
    std::cout  << "construct" << std::endl;
  }

  Data(const Data& obj){
    std::cout  << "copy construct" << std::endl;
  }
  ~Data(){
    std::cout << "deconstruct" << std::endl;
  }
private:
  int A;
  float B;
};

int main()
{
//    ControlCmd cmd;
//    cmd.display("Example");
//    ParkingPoint pp;

    dcom::VectorArray<Data> arrays(100);
    arrays.emplace_back(10, 20);

    dcom::VectorArray<int> arrays2(200, 200);
    for(int i=0; i<5000*5000; ++i){
        arrays2.push_back(i);
    }

    std::vector<int> a;
    std::cerr << a.back() << std::endl;






    return 0;
}

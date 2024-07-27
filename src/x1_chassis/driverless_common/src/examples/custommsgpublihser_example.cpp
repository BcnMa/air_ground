
#include <unistd.h>
#include <driverless_common/tools/lcm_costom_msg_publisher.hpp>


int main()
{
    dcom::LcmCostomMsgPublisher state_publisher;
    state_publisher.init("example", "random", 3);

    float value1= 0, value2 = 0, value3 = 0;
    while(1){

        value1 += 0.1;
        value2 += 0.2;
        value3 += 0.5;

        if(value1 > 1.0){
            value1 = value2 = value3 = 0.0;
        }

        state_publisher.setData(0, "value1", value1, "km/h");
        state_publisher.setData(1, "value2", value2, "rad");
        state_publisher.setData(2, "value3", value3, "");
        state_publisher.publish();
        state_publisher.print();

        usleep(100000);
    }





    return 0;
}

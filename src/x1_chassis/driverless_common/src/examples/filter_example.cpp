
#include <unistd.h>
#include <iostream>
#include <driverless_common/algorithm/filter.h>


int main()
{
    dcom::MeanFilter mean_filter(5);
    for(int i=0; i<100; ++i){
        std::cout << mean_filter.filter(i) << std::endl;
        if(i == 10){
            mean_filter.reset();
        }
    }

    return 0;
}

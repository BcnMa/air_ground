#include <driverless_common/tools/dynamic_reconfigure.hpp>

typedef struct {
    int a, b, c;
}Data;
Data d;

class DynamicReconfigureServerExample{
public:
    void newParamCallback(const exlcm::DynamicReconfigureMsg *msg){
        std::cout << " " << std::endl;
//        for(int i=0; i<msg->param_cnt; ++i){
//            std::cout << "raw: " << msg->param_names[i] << ": " << msg->params[i] << std::endl;
//        }
        std::cout << "========= New parameters received! ===========" << std::endl;
        std::cout << "float_data: " <<  server.getDouble(msg, 0) << std::endl;
        std::cout << "int_data: " <<  server.getInt(msg, 1) << std::endl;
        std::cout << "bool_data: " <<  server.getBool(msg, 2) << std::endl;
        std::cout << "str_data: " <<  server.getString(msg, 3) << std::endl;
        std::cout << std::endl;
    }

    DynamicReconfigureServerExample():
        server("example", "test", 4){
        server.addParam(0, "float_data", 15.0);
        server.addParam(1, "int_data", 8);
        server.addParam(2, "bool_data", true);
        server.addParam(3, "str_data", "string1");
        std::cout << "Listening for new parameters..." << std::endl;
        server.registerCallback(&DynamicReconfigureServerExample::newParamCallback, this);
        server.start();
    }
private:
    dcom::DynamicReconfigureServer server;
    float float_data;
    int int_data;
    bool bool_data;
    std::string str_data;
};


class DynamicReconfigureClientExample{
public:
    void serverReportMsgCallback(const exlcm::DynamicReconfigureMsg *msg){
        std::cout << "serverReportMsgCallback: " << msg->config_channel << "\t" << msg->param_cnt << std::endl;
    }

    DynamicReconfigureClientExample(){
        client.registerCallback(&DynamicReconfigureClientExample::serverReportMsgCallback, this);
        client.start();
    }
    ~DynamicReconfigureClientExample(){
        client.stop();
    }

private:
    dcom::DynamicReconfigureClient client;
    float float_data;
    int int_data;
    bool bool_data;
    std::string str_data;
};

int main(){
    DynamicReconfigureServerExample se;
    DynamicReconfigureClientExample* ce = nullptr;
    int i=0;
    while(1){
        sleep(1);

        continue;

        // 删除测试
        if(i++ % 2 == 0){
            if(ce != nullptr){
                delete ce;
            }
            ce = new DynamicReconfigureClientExample;
        }
    }

    return 0;
}

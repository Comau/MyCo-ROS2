#ifndef MYCO_ETHERCAT_IO_CLIENT_H
#define MYCO_ETHERCAT_IO_CLIENT_H

#include<rclcpp/rclcpp.hpp>
#include <vector>
#include <myco_ethercat_driver/myco_ethercat_manager.h>
#include "myco_robot_msgs/srv/myco_iod_read.hpp"
#include "myco_robot_msgs/srv/myco_iod_write.hpp"
#include <std_srvs/srv/set_bool.hpp>

#include <pthread.h>
#include <time.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace myco_io_txpdo {

const int DIGITAL_INPUT=0;
const int ANALOG_INPUT_CHANNEL1=1;
const int ANALOG_INPUT_CHANNEL2=2;
const int SMART_CAMERA_X=3;
const int SMART_CAMERA_Y=4;

}

namespace myco_io_rxpdo {

const int DIGITAL_OUTPUT=0;

}

namespace  myco_ethercat_driver {

class MycoEtherCATIOClient{

private:
    EtherCatManager* manager_;

    std::shared_ptr<rclcpp::Node> n_, io_nh_;
    std::vector<MycoPDOunit> pdo_input; // txpdo
    std::vector<MycoPDOunit> pdo_output; //rxpdo
    int slave_no_;
    int32_t led_output[8]={0,4096, 8192, 12288, 16384, 20480, 24576,28672};

    rclcpp::Service<myco_robot_msgs::srv::MycoIODRead>::SharedPtr read_sdo_;
    rclcpp::Service<myco_robot_msgs::srv::MycoIODRead>::SharedPtr read_do_;
    rclcpp::Service<myco_robot_msgs::srv::MycoIODWrite>::SharedPtr write_sdo_;
    rclcpp::Service<myco_robot_msgs::srv::MycoIODWrite>::SharedPtr led_control;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_txsdo_server_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_rxsdo_server_;


public:
    MycoEtherCATIOClient(EtherCatManager* manager, int slave_no, const rclcpp::Node::SharedPtr& nh, std::string io_port_name);
    ~MycoEtherCATIOClient();
    int32_t readSDO_unit(int n); // 20201117
    int32_t readDO_unit(int n); // 20201130
    void writeOutput_unit(int n, int32_t val);
    int32_t writeSDO_unit(int n); // 20201117
    
    int16_t readInput_unit(int n);
    int32_t readOutput_unit(int n);
    
    std::string getTxSDO();
    std::string getRxSDO();

    bool readSDO_cb(const std::shared_ptr<myco_robot_msgs::srv::MycoIODRead::Request> req, const std::shared_ptr<myco_robot_msgs::srv::MycoIODRead::Response> resp); // 20201117
    bool readDO_cb(const std::shared_ptr<myco_robot_msgs::srv::MycoIODRead::Request> req, const std::shared_ptr<myco_robot_msgs::srv::MycoIODRead::Response> resp); // 20201130
    bool writeSDO_cb(const std::shared_ptr<myco_robot_msgs::srv::MycoIODWrite::Request> req, const std::shared_ptr<myco_robot_msgs::srv::MycoIODWrite::Response> resp); // 20201117
    bool getRxSDO_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool getTxSDO_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);

    bool writeDO_cb(const std::shared_ptr<myco_robot_msgs::srv::MycoIODWrite::Request> req, const std::shared_ptr<myco_robot_msgs::srv::MycoIODWrite::Response> resp);
    bool readDI_cb(const std::shared_ptr<myco_robot_msgs::srv::MycoIODRead::Request> req, const std::shared_ptr<myco_robot_msgs::srv::MycoIODRead::Response> resp);
    bool led_cb(const std::shared_ptr<myco_robot_msgs::srv::MycoIODWrite::Request> req, const std::shared_ptr<myco_robot_msgs::srv::MycoIODWrite::Response> resp);

};

}

#endif

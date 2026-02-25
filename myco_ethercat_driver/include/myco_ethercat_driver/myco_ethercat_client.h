#ifndef MYCO_ETHERCAT_CLIENT_H
#define MYCO_ETHERCAT_CLIENT_H

#include<rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <vector>
#include <myco_ethercat_driver/myco_ethercat_manager.h>

#include <pthread.h>
#include <time.h>

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <chrono>

namespace myco_txpdo
{
    const int AXIS1_STATUSWORD_L16=0;
    const int AXIS1_ACTTORQUE_H16=0;
    const int AXIS1_ACTPOSITION=1;
    const int AXIS1_ACTVELOCITY_L16=2;
    const int AXIS1_ERRORCODE_L16=3;
    const int AXIS1_MODES_OF_OPERATION_DISPLAY_BYTE2=3;

    const int AXIS2_STATUSWORD_L16=4;
    const int AXIS2_ACTTORQUE_H16=4;
    const int AXIS2_ACTPOSITION=5;
    const int AXIS2_ACTVELOCITY_L16=6;
    const int AXIS2_ERRORCODE_L16=7;
    const int AXIS2_MODES_OF_OPERATION_DISPLAY_BYTE2=7;
}

namespace myco_rxpdo
{
    const int AXIS1_CONTROLWORD_L16=0;
    const int AXIS1_MODES_OF_OPERATION_BYTE2=0;
    const int AXIS1_TARGET_POSITION=1;
    const int AXIS1_TARGET_TORQUE_L16=2;
    const int AXIS1_VELFF_H16=2;

    const int AXIS2_CONTROLWORD_L16=3;
    const int AXIS2_MODES_OF_OPERATION_BYTE2=3;
    const int AXIS2_TARGET_POSITION=4;
    const int AXIS2_TARGET_TORQUE_L16=5;
    const int AXIS2_VELFF_H16=5;
}


namespace myco_ethercat_driver
{
class MycoEtherCATClient
{
private:
    EtherCatManager* manager_; // old namespace

    std::shared_ptr<rclcpp::Node> n_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_input_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_output_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_enable_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_reset_fault_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_open_brake_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_close_brake_;
    std_msgs::msg::String txpdo_msg_;
    std_msgs::msg::String rxpdo_msg_;


    std::vector<MycoPDOunit> pdo_input; // txpdo old namespace
    std::vector<MycoPDOunit> pdo_output; //rxpdo old namespace
    int slave_no_;

public:
    MycoEtherCATClient(EtherCatManager* manager, int slave_no, const rclcpp::Node::SharedPtr& node); // old namespace
    ~MycoEtherCATClient();
    int32_t readInput_unit(int n);
    int32_t readOutput_unit(int n);
    void writeOutput_unit(int n, int32_t val);
    int16_t readInput_half_unit(int n, bool high_16);
    int16_t readOutput_half_unit(int n, bool high_16);
    void writeOutput_half_unit(int n, int16_t val, bool high_16);
    int8_t readInput_unit_byte(int n, bool high_16, bool high_8);
    int8_t readOutput_unit_byte(int n, bool high_16, bool high_8);
    void writeOutput_unit_byte(int n, int8_t val, bool high_16, bool high_8);
    int32_t getAxis1PosCnt();
    int32_t getAxis2PosCnt();
    void setAxis1PosCnt(int32_t pos_cnt);
    void setAxis2PosCnt(int32_t pos_cnt);
    int16_t getAxis1VelCnt();
    int16_t getAxis2VelCnt();
    void setAxis1VelFFCnt(int16_t vff_cnt);
    void setAxis2VelFFCnt(int16_t vff_cnt);
    int16_t getAxis1TrqCnt();
    int16_t getAxis2TrqCnt();
    void setAxis1TrqCnt(int16_t trq_cnt);
    void setAxis2TrqCnt(int16_t trq_cnt);
    void readInput();
    void readOutput();
    void writeOutput();
    std::string getTxPDO();
    std::string getRxPDO();
    std::string getCurrentPosition();
    void getActPosCounts(int32_t &pos_act_count_1, int32_t &pos_act_count_2);
    void getCmdPosCounts(int32_t &pos_cmd_count_1, int32_t &pos_cmd_count_2);
    void pubInput();
    void pubOutput();
    void clearPoseFault();
    bool recognizePose();
    bool isEnabled();

    static void *setEnable(void *threadarg);
    static void *setDisable(void *threadarg);
    static void *recognizePoseCmd(void *threadarg);
    bool isWarning();
    void resetFault();
    bool inPosMode();
    bool inTrqMode();
    bool inPosBasedMode();
    void setPosMode();
    void setTrqMode();
    bool enable_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool reset_fault_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool open_brake_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool close_brake_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
};
}
#endif

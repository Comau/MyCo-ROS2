#ifndef MYCO_ETHERCAT_DRIVER_H
#define MYCO_ETHERCAT_DRIVER_H

#include <myco_ethercat_driver/myco_ethercat_client.h>
#include <myco_ethercat_driver/myco_ethercat_io_client.h>

namespace myco_ethercat_driver {

class MycoEtherCATDriver
{
public:
    MycoEtherCATDriver(EtherCatManager *manager, std::string driver_name,const rclcpp::Node::SharedPtr& node);
    ~MycoEtherCATDriver();

    std::shared_ptr<rclcpp::Node> root_nh_, ed_nh_;

    bool getEnableState();
    bool getFaultState();
    bool getMotionState();
    bool getPosAlignState();
    void updateStatus();

    size_t getEtherCATClientNumber();
    MycoEtherCATClient* getEtherCATClientPtr(size_t n);
    std::string getJointName(size_t n);
    double getReductionRatio(size_t n);
    double getAxisPositionFactor(size_t n);
    double getAxisTorqueFactor(size_t n);
    int32_t getCountZero(size_t n);

    bool recognizePosition();

    bool getTxPDO_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool getRxPDO_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool getCurrentPosition_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool getMotionState_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool getPosAlignState_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool enableRobot_test();
    bool enableRobot_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool disableRobot_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool clearFault_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool recognizePosition_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);

    static int32_t getIntFromStr(std::string str);
    void error_log(int line, std::string log, std::string log_param);

private:
        std::vector<MycoEtherCATClient*> ethercat_clients_;
        std::vector<int64_t> slave_no_;
        std::vector<std::string> joint_names_;
        std::vector<double> reduction_ratios_;
        std::vector<double> axis_position_factors_;
        std::vector<double> axis_torque_factors_;
        std::vector<int64_t> count_zeros_;

        std::vector<MycoEtherCATIOClient*> ethercat_io_clients_;
        std::vector<int64_t> io_slave_no_;

        std::string driver_name_;
        
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_txpdo_server_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_rxpdo_server_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_current_position_server_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_motion_state_server_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_pos_align_state_server_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_robot_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr disable_robot_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clear_fault_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr recognize_position_;

        std_msgs::msg::Bool enable_state_msg_;
        std_msgs::msg::Bool fault_state_msg_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_state_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_state_pub_;
        rclcpp::TimerBase::SharedPtr status_timer_;

        std::vector<double> count_rad_factors_;
        double motion_threshold_;
        double pos_align_threshold_;
};

}

#endif

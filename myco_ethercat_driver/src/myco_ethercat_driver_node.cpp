#include <myco_ethercat_driver/myco_ethercat_driver.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr myco_ethercat_node = rclcpp::Node::make_shared("myco_ethercat_node");
    myco_ethercat_driver::EtherCatManager em("enp2s0");
    myco_ethercat_driver::MycoEtherCATDriver ed(&em, "myco", myco_ethercat_node);
    RCLCPP_INFO(myco_ethercat_node->get_logger(),"myco_ethercat_node start");
    rclcpp::spin(myco_ethercat_node);
}
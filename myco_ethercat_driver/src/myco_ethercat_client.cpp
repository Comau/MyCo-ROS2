#include <myco_ethercat_driver/myco_ethercat_client.h>

namespace myco_ethercat_driver
{
MycoEtherCATClient::MycoEtherCATClient(EtherCatManager *manager, int slave_no,const rclcpp::Node::SharedPtr& node):
    manager_(manager), slave_no_(slave_no), n_(node)
{
    std::string info_tx_name="myco_module_info_tx_slave";
    std::string info_rx_name="myco_module_info_rx_slave";
    std::string enable_server_name="myco_module_enable_slave";
    std::string reset_fault_server_name="myco_module_reset_fault_slave";
    std::string open_brake_server_name="myco_module_open_brake_slave";
    std::string close_brake_server_name="myco_module_close_brake_slave";

    std::string slave_num=boost::lexical_cast<std::string>(slave_no);

    info_tx_name.append(slave_num);
    info_rx_name.append(slave_num);
    enable_server_name.append(slave_num);
    reset_fault_server_name.append(slave_num);
    open_brake_server_name.append(slave_num);
    close_brake_server_name.append(slave_num);

    pub_input_ = n_->create_publisher<std_msgs::msg::String>("info_tx_name", 1);
    pub_output_ = n_->create_publisher<std_msgs::msg::String>("info_rx_name", 1);

    server_enable_=n_->create_service<std_srvs::srv::SetBool>(enable_server_name, std::bind(&MycoEtherCATClient::enable_cb, this,std::placeholders::_1,std::placeholders::_2));
    server_reset_fault_=n_->create_service<std_srvs::srv::SetBool>(reset_fault_server_name, std::bind(&MycoEtherCATClient::reset_fault_cb, this,std::placeholders::_1,std::placeholders::_2));
    server_open_brake_=n_->create_service<std_srvs::srv::SetBool>(open_brake_server_name, std::bind(&MycoEtherCATClient::open_brake_cb, this,std::placeholders::_1,std::placeholders::_2));
    server_close_brake_=n_->create_service<std_srvs::srv::SetBool>(close_brake_server_name, std::bind(&MycoEtherCATClient::close_brake_cb, this,std::placeholders::_1,std::placeholders::_2));

    // init pdo_input and output
    std::string name_pdo_input[8]={"Axis1_Statusword and Axis1_Torque_Actual_Value", "Axis1_Position_Actual_Value",
                                   "Axis1_Velocity_Actual_Value", "Axis1_ErrorCode and Axis1_Modes_of_operation_display",
                                   "Axis2_Statusword and Axis2_Torque_Actual_Value", "Axis2_Position_Actual_Value",
                                   "Axis2_Velocity_Actual_Value", "Axis2_ErrorCode and Axis2_Modes_of_operation_display"};
    uint8_t channel_pdo_input[8]={0, 4, 8, 12, 32, 36, 40, 44};
    pdo_input.clear();
    MycoPDOunit unit_tmp; // old namespace
    for(unsigned i=0; i<8; ++i)
    {
        unit_tmp.name=name_pdo_input[i];
        unit_tmp.channel=channel_pdo_input[i];
        pdo_input.push_back(unit_tmp);
    }

    std::string name_pdo_output[6]={"Axis1_Controlword and Axis1_Modes_of_operation", "Axis1_Target_position", "Axis1_Target_Torque and Axis1_VelFF",
                                    "Axis2_Controlword and Axis2_Modes_of_operation", "Axis2_Target_position", "Axis2_Target_Torque and Axis2_VelFF"};
    uint8_t channel_pdo_output[6]={0, 4, 8, 32, 36, 40};
    pdo_output.clear();
    for(unsigned i=0; i<6; ++i)
    {
        unit_tmp.name=name_pdo_output[i];
        unit_tmp.channel=channel_pdo_output[i];
        pdo_output.push_back(unit_tmp);
    }

    manager_->writeSDO<int8_t>(slave_no, 0x1c12, 0x00, 0x00);
    manager_->writeSDO<int16_t>(slave_no, 0x1c12, 0x01, 0x1600);
    manager_->writeSDO<int16_t>(slave_no, 0x1c12, 0x02, 0x1610);
    manager_->writeSDO<int8_t>(slave_no, 0x1c12, 0x00, 0x02);

    manager_->writeSDO<int8_t>(slave_no, 0x1c13, 0x00, 0x00);
    manager_->writeSDO<int16_t>(slave_no, 0x1c13, 0x01, 0x1a00);
    manager_->writeSDO<int16_t>(slave_no, 0x1c13, 0x02, 0x1a10);
    manager_->writeSDO<int8_t>(slave_no, 0x1c13, 0x00, 0x02);

    writeOutput_half_unit(myco_rxpdo::AXIS1_CONTROLWORD_L16, 0x0, false);
    writeOutput_half_unit(myco_rxpdo::AXIS2_CONTROLWORD_L16, 0x0, false);
    writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
    writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
}

MycoEtherCATClient::~MycoEtherCATClient()
{

}

int32_t MycoEtherCATClient::readInput_unit(int n)
{
    if(n<0 || n>=pdo_input.size())
        return 0x0000;
    uint8_t map[4];
    for(int i=0; i<4; i++)
    {
        map[i]=manager_->readInput(slave_no_, pdo_input[n].channel+i);
    }
    int32_t value_tmp=*(int32_t *)(map);
    return value_tmp;
}

int32_t MycoEtherCATClient::readOutput_unit(int n)
{
    if(n<0 || n>=pdo_output.size())
        return 0x0000;
    uint8_t map[4];
    for(int i=0; i<4; i++)
    {
        map[i]=manager_->readOutput(slave_no_, pdo_output[n].channel+i);
    }
    int32_t value_tmp=*(int32_t *)(map);
    return value_tmp;
}

void MycoEtherCATClient::writeOutput_unit(int n, int32_t val)
{
    if(n<0 || n>=pdo_output.size())
        return;
    uint8_t map_tmp;
    for(int i=0; i<4; i++)
    {
        map_tmp=(val>>8*i) & 0x00ff;
        manager_->write(slave_no_, pdo_output[n].channel+i, map_tmp);
    }
}

int16_t MycoEtherCATClient::readInput_half_unit(int n, bool high_16)
{
    if(n<0 || n>=pdo_input.size())
        return 0x0000;
    int offset;
    if(high_16)
        offset=2;
    else
        offset=0;
    uint8_t map[2];
    for(int i=0; i<2; i++)
    {
        map[i]=manager_->readInput(slave_no_, pdo_input[n].channel+offset+i);
    }
    int16_t value_tmp=*(int16_t *)(map);
    return value_tmp;
}

int16_t MycoEtherCATClient::readOutput_half_unit(int n, bool high_16)
{
    if(n<0 || n>=pdo_output.size())
        return 0x0000;
    int offset;
    if(high_16)
        offset=2;
    else
        offset=0;
    uint8_t map[2];
    for(int i=0; i<2; i++)
    {
        map[i]=manager_->readOutput(slave_no_, pdo_output[n].channel+offset+i);
    }
    int16_t value_tmp=*(int16_t *)(map);
    return value_tmp;
}

void MycoEtherCATClient::writeOutput_half_unit(int n, int16_t val, bool high_16)
{
    if(n<0 || n>=pdo_output.size())
        return;
    int offset;
    if(high_16)
        offset=2;
    else
        offset=0;
    uint8_t map_tmp;
    for(int i=0; i<2; i++)
    {
        map_tmp=(val>>8*i) & 0x00ff;
        manager_->write(slave_no_, pdo_output[n].channel+offset+i, map_tmp);
    }
}

int8_t MycoEtherCATClient::readInput_unit_byte(int n, bool high_16, bool high_8)
{
    if(n<0 || n>=pdo_input.size())
        return 0x0000;
    int offset;
    if(high_16)
    {
        if(high_8)
            offset=3;
        else
            offset=2;
    }
    else
    {
        if(high_8)
            offset=1;
        else
            offset=0;
    }
    uint8_t map[1];
    for(int i=0; i<1; i++)
    {
        map[i]=manager_->readInput(slave_no_, pdo_input[n].channel+offset+i);
    }
    int8_t value_tmp=*(int8_t *)(map);
    return value_tmp;
}

int8_t MycoEtherCATClient::readOutput_unit_byte(int n, bool high_16, bool high_8)
{
    if(n<0 || n>=pdo_output.size())
        return 0x0000;
    int offset;
    if(high_16)
    {
        if(high_8)
            offset=3;
        else
            offset=2;
    }
    else
    {
        if(high_8)
            offset=1;
        else
            offset=0;
    }
    uint8_t map[1];
    for(int i=0; i<1; i++)
    {
        map[i]=manager_->readOutput(slave_no_, pdo_output[n].channel+offset+i);
    }
    int8_t value_tmp=*(int8_t *)(map);
    return value_tmp;
}

void MycoEtherCATClient::writeOutput_unit_byte(int n, int8_t val, bool high_16, bool high_8)
{
    if(n<0 || n>=pdo_output.size())
        return;
    int offset;
    if(high_16)
    {
        if(high_8)
            offset=3;
        else
            offset=2;
    }
    else
    {
        if(high_8)
            offset=1;
        else
            offset=0;
    }
    uint8_t map_tmp;
    for(int i=0; i<1; i++)
    {
        map_tmp=(val>>8*i) & 0x00ff;
        manager_->write(slave_no_, pdo_output[n].channel+offset+i, map_tmp);
    }
}

int32_t MycoEtherCATClient::getAxis1PosCnt()
{
    return readInput_unit(myco_txpdo::AXIS1_ACTPOSITION);
}

int32_t MycoEtherCATClient::getAxis2PosCnt()
{
    return readInput_unit(myco_txpdo::AXIS2_ACTPOSITION);
}

void MycoEtherCATClient::setAxis1PosCnt(int32_t pos_cnt)
{
    writeOutput_unit(myco_rxpdo::AXIS1_TARGET_POSITION, pos_cnt);
}

void MycoEtherCATClient::setAxis2PosCnt(int32_t pos_cnt)
{
    writeOutput_unit(myco_rxpdo::AXIS2_TARGET_POSITION, pos_cnt);
}

int16_t MycoEtherCATClient::getAxis1VelCnt()
{
    return readInput_half_unit(myco_txpdo::AXIS1_ACTVELOCITY_L16, false);
}

int16_t MycoEtherCATClient::getAxis2VelCnt()
{
    return readInput_half_unit(myco_txpdo::AXIS2_ACTVELOCITY_L16, false);
}

void MycoEtherCATClient::setAxis1VelFFCnt(int16_t vff_cnt)
{
    writeOutput_half_unit(myco_rxpdo::AXIS1_VELFF_H16, vff_cnt, true);
}

void MycoEtherCATClient::setAxis2VelFFCnt(int16_t vff_cnt)
{
    writeOutput_half_unit(myco_rxpdo::AXIS2_VELFF_H16, vff_cnt, true);
}

int16_t MycoEtherCATClient::getAxis1TrqCnt()
{
    return readInput_half_unit(myco_txpdo::AXIS1_ACTTORQUE_H16, true);
}

int16_t MycoEtherCATClient::getAxis2TrqCnt()
{
    return readInput_half_unit(myco_txpdo::AXIS2_ACTTORQUE_H16, true);
}

void MycoEtherCATClient::setAxis1TrqCnt(int16_t trq_cnt)
{
    writeOutput_half_unit(myco_rxpdo::AXIS1_TARGET_TORQUE_L16, trq_cnt, false);
}

void MycoEtherCATClient::setAxis2TrqCnt(int16_t trq_cnt)
{
    writeOutput_half_unit(myco_rxpdo::AXIS2_TARGET_TORQUE_L16, trq_cnt, false);
}

void MycoEtherCATClient::readInput()
{
    for(int i=0; i<pdo_input.size(); i++)
    {
        readInput_unit(i);
    }
}

void MycoEtherCATClient::readOutput()
{
    for(int i=0; i<pdo_output.size(); i++)
    {
        readOutput_unit(i);
    }
}

std::string MycoEtherCATClient::getTxPDO()
{
    int length=64;
    uint8_t map[length];
    char temp[8];
    std::string result="slave";
    result.reserve(640); // the size of result is actually 410
    std::string slave_num=boost::lexical_cast<std::string>(slave_no_);
    result.append(slave_num);
    result.append("_txpdo:\n");
    for (unsigned i = 0; i < length; ++i)
    {
        map[i] = manager_->readInput(slave_no_, i);
        sprintf(temp,"0x%.2x",(uint8_t)map[i]);
        result.append(temp, 4);
        result.append(":");
    }
    result.append("\n");
    return result;
}

std::string MycoEtherCATClient::getRxPDO()
{
    int length=64;
    uint8_t map[length];
    char temp[8];
    std::string result="slave";
    result.reserve(640); // the size of result is actually 410
    std::string slave_num=boost::lexical_cast<std::string>(slave_no_);
    result.append(slave_num);
    result.append("_rxpdo:\n");
    for (unsigned i = 0; i < length; ++i)
    {
        map[i] = manager_->readOutput(slave_no_, i);
        sprintf(temp,"0x%.2x",(uint8_t)map[i]);
        result.append(temp, 4);
        result.append(":");
    }
    result.append("\n");
    return result;
}

std::string MycoEtherCATClient::getCurrentPosition()
{
    std::string result="slave";
    std::string slave_num=boost::lexical_cast<std::string>(slave_no_);
    result.append(slave_num);
    result.append("_current_position:\n");
    int32_t current_position_1=readInput_unit(myco_txpdo::AXIS1_ACTPOSITION);
    std::string tmp_str_1=boost::lexical_cast<std::string>(current_position_1);
    result.append("axis1: ");
    result.append(tmp_str_1);
    result.append(", ");
    int32_t current_position_2=readInput_unit(myco_txpdo::AXIS2_ACTPOSITION);
    std::string tmp_str_2=boost::lexical_cast<std::string>(current_position_2);
    result.append("axis2: ");
    result.append(tmp_str_2);
    result.append(". \n");
    return result;
}

void MycoEtherCATClient::getActPosCounts(int32_t &pos_act_count_1, int32_t &pos_act_count_2)
{
    pos_act_count_1=readInput_unit(myco_txpdo::AXIS1_ACTPOSITION);
    pos_act_count_2=readInput_unit(myco_txpdo::AXIS2_ACTPOSITION);
}

void MycoEtherCATClient::getCmdPosCounts(int32_t &pos_cmd_count_1, int32_t &pos_cmd_count_2)
{
    pos_cmd_count_1=readOutput_unit(myco_rxpdo::AXIS1_TARGET_POSITION);
    pos_cmd_count_2=readOutput_unit(myco_rxpdo::AXIS2_TARGET_POSITION);
}

void MycoEtherCATClient::pubInput()
{
    txpdo_msg_.data=getTxPDO();
    pub_input_->publish(txpdo_msg_);
    txpdo_msg_.data.clear();
}

void MycoEtherCATClient::pubOutput()
{
    rxpdo_msg_.data=getRxPDO();
    pub_output_->publish(rxpdo_msg_);
    rxpdo_msg_.data.clear();
}

void MycoEtherCATClient::clearPoseFault()
{
    // channel1
    writeOutput_half_unit(myco_rxpdo::AXIS1_CONTROLWORD_L16, 0x86, false);
    usleep(20000);
    writeOutput_half_unit(myco_rxpdo::AXIS1_CONTROLWORD_L16, 0x6, false);
    usleep(20000);

    // channel2
    writeOutput_half_unit(myco_rxpdo::AXIS2_CONTROLWORD_L16, 0x86, false);
    usleep(20000);
    writeOutput_half_unit(myco_rxpdo::AXIS2_CONTROLWORD_L16, 0x6, false);
    usleep(20000);
}

bool MycoEtherCATClient::recognizePose()
{
    //channel1
    if((readInput_half_unit(myco_txpdo::AXIS1_STATUSWORD_L16, false) & 0xc) == 0)
    {
        manager_->writeSDO<int8_t>(slave_no_, 0x6060, 0x0, 0xc);
        writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0xc, true, false);
        usleep(20000);
        manager_->writeSDO<int32_t>(slave_no_, 0x3024, 0x0, 0x11000000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(rclcpp::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2023, 0x0)==0x200000
               && manager_->readSDO<int32_t>(slave_no_, 0x2024, 0x0)==0x200000)
            {
                manager_->writeSDO<int32_t>(slave_no_, 0x3024, 0x0, 0x33000000);
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 20e+9)
            {
                double result = (manager_->readSDO<int32_t>(slave_no_, 0x2043, 0x0));
                result = result/4096/2.7*49.7*3.3;
                fprintf(stderr,"The voltage of slave %i is: %fV.\n",slave_no_, result);
                // ROS_WARN("recognizePose phase1 failed while pose recognition in slave %i, channel 1", slave_no_);
                RCLCPP_WARN(n_->get_logger(), "recognizePose phase1 failed while pose recognition in slave %i, channel 1", slave_no_);
                writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return false;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(rclcpp::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2023, 0x0)==0
               && manager_->readSDO<int32_t>(slave_no_, 0x2024, 0x0)==0)
            {
                manager_->writeSDO<int32_t>(slave_no_, 0x3024, 0x0, 0x0);
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 5e+9)
            {
                // ROS_WARN("recognizePose phase 2 failed while pose recognition in slave %i, channel 1", slave_no_);
                RCLCPP_WARN(n_->get_logger(),"recognizePose phase 2 failed while pose recognition in slave %i, channel 1", slave_no_);
                writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return false;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        manager_->writeSDO<int8_t>(slave_no_, 0x6060, 0x0, 0x8);
        writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        usleep(50000);
    }
    else
    {
        // ROS_WARN("recognizePose failed in slave %i, channel 1, the reason might be there is a fault or the motor is enabled", slave_no_);
        RCLCPP_WARN(n_->get_logger(),"recognizePose failed in slave %i, channel 1, the reason might be there is a fault or the motor is enabled", slave_no_);
        return false;
    }

    //channel2
    if((readInput_half_unit(myco_txpdo::AXIS2_STATUSWORD_L16, false) & 0xc) == 0)
    {
        manager_->writeSDO<int8_t>(slave_no_, 0x6860, 0x0, 0xc);
        writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0xc, true, false);
        usleep(20000);
        manager_->writeSDO<int32_t>(slave_no_, 0x3034, 0x0, 0x11000000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(rclcpp::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2033, 0x0)==0x200000
               && manager_->readSDO<int32_t>(slave_no_, 0x2034, 0x0)==0x200000)
            {
                manager_->writeSDO<int32_t>(slave_no_, 0x3034, 0x0, 0x33000000);
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 20e+9)
            {
                // ROS_WARN("recognizePose phase1 failed while pose recognition in slave %i, channel 2", slave_no_);
                RCLCPP_WARN(n_->get_logger(),"recognizePose phase1 failed while pose recognition in slave %i, channel 2", slave_no_);
                writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return false;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(rclcpp::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2033, 0x0)==0
               && manager_->readSDO<int32_t>(slave_no_, 0x2034, 0x0)==0)
            {
                manager_->writeSDO<int32_t>(slave_no_, 0x3034, 0x0, 0x0);
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 5e+9)
            {
                // ROS_WARN("recognizePose phase 2 failed while pose recognition in slave %i, channel 2", slave_no_);
                RCLCPP_WARN(n_->get_logger(),"recognizePose phase 2 failed while pose recognition in slave %i, channel 2", slave_no_);
                writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return false;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        manager_->writeSDO<int8_t>(slave_no_, 0x6860, 0x0, 0x8);
        writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        usleep(50000);
    }
    else
    {
        // ROS_WARN("recognizePose failed in slave %i, channel 2, the reason might be there is a fault or the motor is enabled", slave_no_);
        RCLCPP_WARN(n_->get_logger(),"recognizePose failed in slave %i, channel 2, the reason might be there is a fault or the motor is enabled", slave_no_);
        return false;
    }

    return true;
}

bool MycoEtherCATClient::isEnabled()
{
    if((readInput_half_unit(myco_txpdo::AXIS1_STATUSWORD_L16, false) & 0xf)==0x7
            && (readInput_half_unit(myco_txpdo::AXIS2_STATUSWORD_L16, false) & 0xf)==0x7)
        return true;
    else
        return false;
}

void *MycoEtherCATClient::setEnable(void* threadarg)
{
    MycoEtherCATClient *pthis=(MycoEtherCATClient *)threadarg;

    if(pthis->readInput_half_unit(myco_txpdo::AXIS1_ERRORCODE_L16, false)==0x2000
       || pthis->readInput_half_unit(myco_txpdo::AXIS2_ERRORCODE_L16, false)==0x2000)
    {
        if(pthis->isWarning())
        {
            pthis->clearPoseFault();
        }

        if(!pthis->recognizePose())
        {
            return (void *)0;
        }
    }

    if(pthis->isWarning())
    {
        pthis->clearPoseFault();
    }

    // enable
    if(pthis->isWarning() || pthis->isEnabled())
    {
        // RCLCPP_WARN(n_->get_logger(),"setEnable in slave %i failed, the reason might be there is a fault or the motor is enabled", pthis->slave_no_);
        std::cout<<"setEnable in slave "<<pthis->slave_no_<<"failed, the reason might be there is a fault or the motor is enabled"<<std::endl;
        return (void *)0;
    }

    pthis->writeOutput_unit(myco_rxpdo::AXIS1_TARGET_POSITION, pthis->readInput_unit(myco_txpdo::AXIS1_ACTPOSITION));
    pthis->writeOutput_unit(myco_rxpdo::AXIS2_TARGET_POSITION, pthis->readInput_unit(myco_txpdo::AXIS2_ACTPOSITION));
    usleep(100000);

    pthis->writeOutput_half_unit(myco_rxpdo::AXIS1_CONTROLWORD_L16, 0x6, false);
    pthis->writeOutput_half_unit(myco_rxpdo::AXIS2_CONTROLWORD_L16, 0x6, false);

    pthis->writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
    pthis->writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);

    struct timespec before, tick;
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while(rclcpp::ok())
    {
        if(pthis->readInput_half_unit(myco_txpdo::AXIS1_STATUSWORD_L16, false)==0x21
           && pthis->readInput_half_unit(myco_txpdo::AXIS2_STATUSWORD_L16, false)==0x21)
        {
            break;
        }
        if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
        {
            // RCLCPP_WARN(n_->get_logger(),"setEnable phase1 in slave %i failed", pthis->slave_no_);
            std::cout<<"setEnable phase1 in slave "<<pthis->slave_no_<<"failed"<<std::endl;
            return (void *)0;
        }
        usleep(10000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }

    pthis->writeOutput_half_unit(myco_rxpdo::AXIS1_CONTROLWORD_L16, 0x7, false);
    pthis->writeOutput_half_unit(myco_rxpdo::AXIS2_CONTROLWORD_L16, 0x7, false);

    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while(rclcpp::ok())
    {
        if(pthis->readInput_half_unit(myco_txpdo::AXIS1_STATUSWORD_L16, false)==0x23
           && pthis->readInput_half_unit(myco_txpdo::AXIS2_STATUSWORD_L16, false)==0x23)
        {
            break;
        }
        if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
        {
            std::cout<<"setEnable phase2 in slave "<<pthis->slave_no_<<"failed"<<std::endl;
            return (void *)0;
        }
        usleep(10000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }

    pthis->writeOutput_half_unit(myco_rxpdo::AXIS1_CONTROLWORD_L16, 0xf, false);
    pthis->writeOutput_half_unit(myco_rxpdo::AXIS2_CONTROLWORD_L16, 0xf, false);

    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while(rclcpp::ok())
    {
        if(pthis->readInput_half_unit(myco_txpdo::AXIS1_STATUSWORD_L16, false)==0x27
           && pthis->readInput_half_unit(myco_txpdo::AXIS2_STATUSWORD_L16, false)==0x27)
        {
            break;
        }
        if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
        {
            std::cout<<"setEnable phase3 in slave "<<pthis->slave_no_<<"failed"<<std::endl;
            return (void *)0;
        }
        usleep(10000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }

    pthis->writeOutput_half_unit(myco_rxpdo::AXIS1_CONTROLWORD_L16, 0x1f, false);
    pthis->writeOutput_half_unit(myco_rxpdo::AXIS2_CONTROLWORD_L16, 0x1f, false);

    usleep(100000);
}

void *MycoEtherCATClient::setDisable(void *threadarg)
{
    MycoEtherCATClient *pthis=(MycoEtherCATClient *)threadarg;
    if(pthis->isEnabled())
    {
        pthis->writeOutput_half_unit(myco_rxpdo::AXIS1_CONTROLWORD_L16, 0x6, false);
        pthis->writeOutput_half_unit(myco_rxpdo::AXIS2_CONTROLWORD_L16, 0x6, false);
        return (void *)0;
    }
    else
    {
        return (void *)0;
    }
}

void *MycoEtherCATClient::recognizePoseCmd(void *threadarg)
{
    MycoEtherCATClient *pthis=(MycoEtherCATClient *)threadarg;

    if(pthis->isWarning())
    {
        pthis->clearPoseFault();
    }

    if(!pthis->recognizePose())
    {
        return (void *)0;
    }

}

bool MycoEtherCATClient::isWarning()
{
    if((readInput_half_unit(myco_txpdo::AXIS1_STATUSWORD_L16, false) & 0x08)==0x08
       || (readInput_half_unit(myco_txpdo::AXIS2_STATUSWORD_L16, false) & 0x08)==0x08)
        return true;
    else
        return false;
}

void MycoEtherCATClient::resetFault()
{
    clearPoseFault();
}

bool MycoEtherCATClient::inPosMode()
{
    if(isEnabled()
       && readInput_unit_byte(myco_txpdo::AXIS1_MODES_OF_OPERATION_DISPLAY_BYTE2, true, false)==0x8
       && readInput_unit_byte(myco_txpdo::AXIS2_MODES_OF_OPERATION_DISPLAY_BYTE2, true, false)==0x8)
        return true;
    else
        return false;
}

bool MycoEtherCATClient::inTrqMode()
{
    if(isEnabled()
       && readInput_unit_byte(myco_txpdo::AXIS1_MODES_OF_OPERATION_DISPLAY_BYTE2, true, false)==0xa
       && readInput_unit_byte(myco_txpdo::AXIS2_MODES_OF_OPERATION_DISPLAY_BYTE2, true, false)==0xa)
        return true;
    else
        return false;
}

bool MycoEtherCATClient::inPosBasedMode()
{
    return inPosMode();
}

void MycoEtherCATClient::setPosMode()
{
    writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
    writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
}

void MycoEtherCATClient::setTrqMode()
{
    writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0xa, true, false);
    writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0xa, true, false);
}

bool MycoEtherCATClient::enable_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    if(!req->data)
    {
        resp->success=false;
        resp->message="require's data is false";
        return true;
    }
    setEnable((void *)this);

    struct timespec before, tick;
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while (rclcpp::ok())
    {
        if(isEnabled())
        {
            resp->success=true;
            resp->message="myco module is enabled";
            return true;
        }
        if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 20e+9)
        {
            resp->success=false;
            resp->message="myco module is not enabled";
            return true;
        }
        usleep(100000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }
}

bool MycoEtherCATClient::reset_fault_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    if(!req->data)
    {
        resp->success=false;
        resp->message="require's data is false";
        return true;
    }

    resetFault();

    resp->success=true;
    resp->message="fault is reset";
    return true;
}

bool MycoEtherCATClient::open_brake_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    if(!req->data)
    {
        resp->success=false;
        resp->message="request's data is false";
        return true;
    }

    //channel1
    if((readInput_half_unit(myco_txpdo::AXIS1_STATUSWORD_L16, false) & 0xc) == 0)
    {
        manager_->writeSDO<int8_t>(slave_no_, 0x6060, 0x0, 0xb);
        writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0xb, true, false);
        usleep(20000);
        manager_->writeSDO<int32_t>(slave_no_, 0x3023, 0x0, 0x11000000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(rclcpp::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2023, 0x0)==0x300000
               && manager_->readSDO<int32_t>(slave_no_, 0x2024, 0x0)==0x300000)
            {
                usleep(20000);
                manager_->writeSDO<int32_t>(slave_no_, 0x3023, 0x0, 0x33000000);
                usleep(30000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp->message="Channel 1 phase 1 failed";
                resp->success=false;
                writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(rclcpp::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2023, 0x0)==0
               && manager_->readSDO<int32_t>(slave_no_, 0x2024, 0x0)==0)
            {
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp->message="Channel 1 phase 2 failed";
                resp->success=false;
                writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
    }
    else
    {
        resp->message="Channel 1 failed, the reason might be there is a fault or the motor is enabled";
        resp->success=false;
        writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        return true;
    }

    //channel2
    if((readInput_half_unit(myco_txpdo::AXIS2_STATUSWORD_L16, false) & 0xc) == 0)
    {
        manager_->writeSDO<int8_t>(slave_no_, 0x6860, 0x0, 0xb);
        writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0xb, true, false);
        usleep(20000);
        manager_->writeSDO<int32_t>(slave_no_, 0x3033, 0x0, 0x11000000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(rclcpp::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2033, 0x0)==0x300000
               && manager_->readSDO<int32_t>(slave_no_, 0x2034, 0x0)==0x300000)
            {
                usleep(20000);
                manager_->writeSDO<int32_t>(slave_no_, 0x3033, 0x0, 0x33000000);
                usleep(30000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp->message="Channel 2 phase 1 failed";
                resp->success=false;
                writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(rclcpp::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2033, 0x0)==0
               && manager_->readSDO<int32_t>(slave_no_, 0x2034, 0x0)==0)
            {
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp->message="Channel 2 phase 2 failed";
                resp->success=false;
                writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
    }
    else
    {
        resp->message="Channel 2 failed, the reason might be there is a fault or the motor is enabled";
        resp->success=false;
        writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        return true;
    }

    resp->message="band-type brake is opened";
    resp->success=true;
    return true;
}

bool MycoEtherCATClient::close_brake_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    if(!req->data)
    {
        resp->success=false;
        resp->message="request's data is false";
        return true;
    }

    //channel1
    if((readInput_half_unit(myco_txpdo::AXIS1_STATUSWORD_L16, false) & 0xc) == 0)
    {
        manager_->writeSDO<int8_t>(slave_no_, 0x6060, 0x0, 0xb);
        writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0xb, true, false);
        usleep(20000);
        manager_->writeSDO<int32_t>(slave_no_, 0x3023, 0x0, 0x22000000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(rclcpp::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2023, 0x0)==0x400000
               && manager_->readSDO<int32_t>(slave_no_, 0x2024, 0x0)==0x400000)
            {
                usleep(20000);
                manager_->writeSDO<int32_t>(slave_no_, 0x3023, 0x0, 0x33000000);
                usleep(30000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp->message="Channel 1 phase 1 failed";
                resp->success=false;
                writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(rclcpp::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2023, 0x0)==0
               && manager_->readSDO<int32_t>(slave_no_, 0x2024, 0x0)==0)
            {
                manager_->writeSDO<int32_t>(slave_no_, 0x3023, 0x0, 0x0);
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp->message="Channel 1 phase 2 failed";
                resp->success=false;
                writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        manager_->writeSDO<int8_t>(slave_no_, 0x6060, 0x0, 0x8);
        writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        usleep(50000);
    }
    else
    {
        resp->message="Channel 1 failed, the reason might be there is a fault or the motor is enabled";
        resp->success=false;
        writeOutput_unit_byte(myco_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        return true;
    }

    //channel2
    if((readInput_half_unit(myco_txpdo::AXIS2_STATUSWORD_L16, false) & 0xc) == 0)
    {
        manager_->writeSDO<int8_t>(slave_no_, 0x6860, 0x0, 0xb);
        writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0xb, true, false);
        usleep(20000);
        manager_->writeSDO<int32_t>(slave_no_, 0x3033, 0x0, 0x22000000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(rclcpp::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2033, 0x0)==0x400000
               && manager_->readSDO<int32_t>(slave_no_, 0x2034, 0x0)==0x400000)
            {
                usleep(20000);
                manager_->writeSDO<int32_t>(slave_no_, 0x3033, 0x0, 0x33000000);
                usleep(30000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp->message="Channel 2 phase 1 failed";
                resp->success=false;
                writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(rclcpp::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2033, 0x0)==0
               && manager_->readSDO<int32_t>(slave_no_, 0x2034, 0x0)==0)
            {
                manager_->writeSDO<int32_t>(slave_no_, 0x3033, 0x0, 0x0);
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp->message="Channel 2 phase 2 failed";
                resp->success=false;
                writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        manager_->writeSDO<int8_t>(slave_no_, 0x6860, 0x0, 0x8);
        writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        usleep(50000);
    }
    else
    {
        resp->message="Channel 2 failed, the reason might be there is a fault or the motor is enabled";
        resp->success=false;
        writeOutput_unit_byte(myco_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        return true;
    }

    resp->message="band-type brake is closed";
    resp->success=true;
    return true;
}

}


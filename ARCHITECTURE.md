# MyCo-ROS2 Driver Architecture

**Complete System Architecture - From High Level to Hardware**

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Layer 1: Application & User Interface](#layer-1-application--user-interface)
3. [Layer 2: ROS2 Control Framework](#layer-2-ros2-control-framework)
4. [Layer 3: EtherCAT Driver Layer](#layer-3-ethercat-driver-layer)
5. [Layer 4: EtherCAT Manager](#layer-4-ethercat-manager)
6. [Layer 5: SOEM Library](#layer-5-soem-library)
7. [Layer 6: Network Layer](#layer-6-network-layer)
8. [Layer 7: Hardware Layer](#layer-7-hardware-layer)
9. [Data Flow](#data-flow)
10. [Configuration & Calibration](#configuration--calibration)
11. [Real-Time Considerations](#real-time-considerations)

---

## Architecture Overview

The MyCo-ROS2 driver implements a 7-layer architecture stack:

```
┌─────────────────────────────────────────────┐
│  MoveIt2 / MycoBasicAPI (Application)      │  Layer 1: High Level
├─────────────────────────────────────────────┤
│  ROS2 Control / MycoHWInterface            │  Layer 2: Control Framework
├─────────────────────────────────────────────┤
│  MycoEtherCATDriver / Client               │  Layer 3: Driver Abstraction
├─────────────────────────────────────────────┤
│  EtherCatManager                           │  Layer 4: Real-time Manager
├─────────────────────────────────────────────┤
│  SOEM (Simple Open EtherCAT Master)        │  Layer 5: Protocol Stack
├─────────────────────────────────────────────┤
│  EtherCAT Protocol / Ethernet              │  Layer 6: Network
├─────────────────────────────────────────────┤
│  Robot Modules / Motor Drives / Encoders   │  Layer 7: Hardware
└─────────────────────────────────────────────┘
```

---

## Layer 1: Application & User Interface

### MoveIt2 Motion Planning

**Purpose:** Provides high-level motion planning, collision avoidance, and trajectory generation.

**Components:**
- **MoveIt2 Core:** Motion planning framework
- **RViz Plugin:** Visualization and interactive planning
- **Planning Pipelines:** OMPL, Pilz, etc.
- **Robot-specific packages:** 
  - `myco_3_590mm_ros2_moveit2`
  - `myco_5_800mm_ros2_moveit2`
  - `myco_8_1300mm_ros2_moveit2`
  - `myco_10_1000mm_ros2_moveit2`
  - `myco_15_1300mm_ros2_moveit2`

**Key Files:**
- Launch files for Gazebo simulation
- MoveIt configuration packages
- URDF/XACRO robot descriptions

### MycoBasicAPI

**Location:** `myco_basic_api/`

**Purpose:** High-level C++/Python API for simplified robot control.

**Architecture:**
```cpp
class MycoBasicAPI {
    MycoMotionAPI*  motion_api_;    // Trajectory planning & execution
    MycoTeleopAPI*  teleop_api_;    // Teleoperation control
    MoveGroupInterface* group_;      // MoveIt interface
}
```

**Key Features:**

1. **Motion Control:**
   - Joint space motion planning
   - Cartesian space motion planning
   - Multi-waypoint path planning
   - Velocity scaling (dynamic reconfigure)

2. **ROS2 Topics (Subscribed):**
   - `/joint_goal` (sensor_msgs/JointState): Joint space target
   - `/cart_goal` (geometry_msgs/PoseStamped): Cartesian target
   - `/cart_path_goal` (geometry_msgs/PoseArray): Path waypoints
   - `/myco_arm_controller/joint_trajectory` (trajectory_msgs/JointTrajectory): Direct trajectory execution
   - `/myco_teleop_joint_cmd_no_limit` (std_msgs/Int64): Developer teleoperation

3. **ROS2 Topics (Published):**
   - `/myco_arm_controller/state`: Joint trajectory controller state
   - `/enable_state`: Servo enable status (true/false)
   - `/fault_state`: Fault/warning status
   - `/reference_link_name`: Current reference frame
   - `/end_link_name`: Current end effector frame

4. **ROS2 Services:**
   - `/get_reference_link`: Query reference link
   - `/get_end_link`: Query end effector link
   - `/stop_teleop`: Emergency stop
   - `/get_txpdo`: Read slave output PDOs
   - `/get_rxpdo`: Read master output PDOs
   - `/get_current_position`: Query joint positions
   - `/recognize_position`: Initialize position recognition
   - `/write_do`: Write digital output
   - `/read_di`: Read digital input
   - `/myco_module_{open|close}_brake_slaveX`: Brake control per module

**GUI Control Panel:**
- Python-based control interface
- Wxwidgets GUI
- Velocity scaling control
- Manual jog controls

---

## Layer 2: ROS2 Control Framework

### MycoHWInterface

**Location:** `myco_ros_control/src/myco_hardware_interface.cpp`

**Purpose:** Bridge between ROS2 controllers and EtherCAT driver.

**Class Definition:**
```cpp
class MycoHWInterface : public hardware_interface::SystemInterface {
private:
    EtherCatManager* em;
    std::vector<MycoEtherCATDriver*> ethercat_drivers_;
    std::vector<ModuleInfo> module_infos_;
    
public:
    // Lifecycle callbacks
    CallbackReturn on_init(const HardwareInfo& info) override;
    CallbackReturn on_activate() override;
    CallbackReturn on_deactivate() override;
    
    // Interface exports
    std::vector<StateInterface> export_state_interfaces() override;
    std::vector<CommandInterface> export_command_interfaces() override;
    
    // Mode switching
    return_type prepare_command_mode_switch(...) override;
    return_type perform_command_mode_switch(...) override;
    
    // Real-time cycle
    return_type read(const Time& time, const Duration& period) override;
    return_type write(const Time& time, const Duration& period) override;
};
```

**Module Information Structure:**
```cpp
typedef struct {
    MycoEtherCATClient* client_ptr;
    AxisInfo axis1;
    AxisInfo axis2;
} ModuleInfo;

typedef struct {
    std::string name;
    double reduction_ratio;           // Gearbox ratio
    double count_rad_factor;          // Count to radian conversion
    double count_rad_per_s_factor;    // Count to rad/s conversion
    double count_Nm_factor;           // Count to Nm conversion
    int32_t count_zero;               // Calibration offset
    
    double axis_position_factor;      // Encoder resolution
    double axis_torque_factor;        // Torque scaling
    
    // Current state
    double position;
    double velocity;
    double effort;
    
    // Commands
    double position_cmd;
    double velocity_cmd;
    double vel_ff_cmd;                // Velocity feedforward
    double effort_cmd;
    
    double position_cmd_last;
} AxisInfo;
```

**Responsibilities:**

1. **Initialization (`on_init`):**
   - Parse hardware configuration
   - Create EtherCatManager instance
   - Initialize EtherCAT drivers for each robot
   - Configure module information with calibration data

2. **State/Command Interfaces:**
   - Export position, velocity, effort state interfaces
   - Export position, velocity command interfaces
   - Per-joint interface registration

3. **Real-time Cycle (`read`/`write` at 250 Hz):**
   ```cpp
   return_type read(const Time& time, const Duration& period) {
       // Read actual positions/velocities from EtherCAT
       for each module:
           getActPosCounts(&pos1, &pos2);
           Convert counts to radians;
           Update state interfaces;
   }
   
   return_type write(const Time& time, const Duration& period) {
       // Write commanded positions/velocities to EtherCAT
       for each module:
           Convert radians to counts;
           setAxis1/2PosCnt(pos_count);
           setAxis1/2VelFFCnt(vel_count);
   }
   ```

4. **Mode Switching:**
   - Detect when controllers switch between position/velocity/effort modes
   - Synchronize with EtherCAT driver mode changes
   - Thread-safe mode switching with mutexes

### ROS2 Controllers

**Controller Manager Configuration:**
- Update rate: 250 Hz (4ms cycle time)
- Controller types:
  - `joint_trajectory_controller/JointTrajectoryController`
  - `joint_state_broadcaster/JointStateBroadcaster`

**Joint Trajectory Controller:**
- Interpolates smooth trajectories
- Monitors goal completion
- Publishes controller state
- Action server for FollowJointTrajectory

**Configuration Example:**
```yaml
controller_manager:
  ros__parameters:
    update_rate: 250

myco_arm_controller:
  ros__parameters:
    joints: [myco_joint1, myco_joint2, ...]
    command_interfaces: [position, velocity]
    state_interfaces: [position, velocity]
    state_publish_rate: 250.0
    action_monitor_rate: 250.0
    constraints:
      stopped_velocity_tolerance: 0.2
      goal_time: 0.6
```

---

## Layer 3: EtherCAT Driver Layer

### MycoEtherCATDriver

**Location:** `myco_ethercat_driver/src/myco_ethercat_driver.cpp`

**Purpose:** High-level driver managing multiple robot modules.

**Class Structure:**
```cpp
class MycoEtherCATDriver {
private:
    std::vector<MycoEtherCATClient*> ethercat_clients_;
    std::vector<MycoEtherCATIOClient*> ethercat_io_clients_;
    
    std::vector<int64_t> slave_no_;
    std::vector<std::string> joint_names_;
    std::vector<double> reduction_ratios_;
    std::vector<double> axis_position_factors_;
    std::vector<double> axis_torque_factors_;
    std::vector<int64_t> count_zeros_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_state_pub_;
    
    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_robot_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr disable_robot_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clear_fault_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr recognize_position_;
    
public:
    MycoEtherCATDriver(EtherCatManager* manager, 
                       std::string driver_name,
                       const rclcpp::Node::SharedPtr& node);
    
    bool getEnableState();
    bool getFaultState();
    bool getMotionState();
    bool recognizePosition();
    
    MycoEtherCATClient* getEtherCATClientPtr(size_t n);
    double getReductionRatio(size_t n);
    // ... accessor methods
};
```

**Responsibilities:**
1. Load configuration from YAML (slave numbers, joint names, calibration)
2. Create EtherCAT client for each module
3. Monitor overall system status (enable, fault, motion)
4. Provide services for robot enable/disable/fault reset
5. Coordinate multi-module operations

### MycoEtherCATClient

**Location:** `myco_ethercat_driver/src/myco_ethercat_client.cpp`

**Purpose:** Control a single robot module (2 axes).

**PDO Mapping:**

**TxPDO (Slave → Master) - Input from drives:**
```cpp
namespace myco_txpdo {
    // Axis 1
    const int AXIS1_STATUSWORD_L16 = 0;        // Status word
    const int AXIS1_ACTTORQUE_H16 = 0;         // Actual torque
    const int AXIS1_ACTPOSITION = 1;            // Actual position (32-bit)
    const int AXIS1_ACTVELOCITY_L16 = 2;       // Actual velocity
    const int AXIS1_ERRORCODE_L16 = 3;         // Error code
    const int AXIS1_MODES_OF_OPERATION_DISPLAY_BYTE2 = 3;
    
    // Axis 2 (similar structure)
    const int AXIS2_STATUSWORD_L16 = 4;
    // ...
}
```

**RxPDO (Master → Slave) - Output to drives:**
```cpp
namespace myco_rxpdo {
    // Axis 1
    const int AXIS1_CONTROLWORD_L16 = 0;       // Control word
    const int AXIS1_MODES_OF_OPERATION_BYTE2 = 0;
    const int AXIS1_TARGET_POSITION = 1;        // Target position (32-bit)
    const int AXIS1_TARGET_TORQUE_L16 = 2;     // Target torque
    const int AXIS1_VELFF_H16 = 2;             // Velocity feedforward
    
    // Axis 2 (similar structure)
    const int AXIS2_CONTROLWORD_L16 = 3;
    // ...
}
```

**Key Methods:**

```cpp
// Position control
int32_t getAxis1PosCnt();              // Read actual position
void setAxis1PosCnt(int32_t pos_cnt);  // Write target position

// Velocity control
int16_t getAxis1VelCnt();              // Read actual velocity
void setAxis1VelFFCnt(int16_t vff_cnt); // Write velocity feedforward

// Torque control
int16_t getAxis1TrqCnt();              // Read actual torque
void setAxis1TrqCnt(int16_t trq_cnt);  // Write target torque

// State machine
bool isEnabled();                       // Check if servo enabled
bool isWarning();                       // Check fault status
void resetFault();                      // Clear fault condition
bool recognizePose();                   // Position recognition

// Mode switching
bool inPosMode();                       // Check if in position mode
bool inTrqMode();                       // Check if in torque mode
void setPosMode();                      // Switch to position mode
void setTrqMode();                      // Switch to torque mode
```

**Control Word (CiA 402 State Machine):**
```
Bit 0: Switch On
Bit 1: Enable Voltage
Bit 2: Quick Stop
Bit 3: Enable Operation
Bit 4-6: Operation mode specific
Bit 7: Fault Reset
Bit 8-15: Manufacturer specific
```

**Status Word:**
```
Bit 0: Ready to switch on
Bit 1: Switched on
Bit 2: Operation enabled
Bit 3: Fault
Bit 4: Voltage enabled
Bit 5: Quick stop
Bit 6: Switch on disabled
Bit 7: Warning
Bit 8-15: Manufacturer specific
```

### MycoEtherCATIOClient

**Location:** `myco_ethercat_driver/src/myco_ethercat_io_client.cpp`

**Purpose:** Handle digital I/O module communication.

**Features:**
- Read digital inputs (DI)
- Write digital outputs (DO)
- Service interfaces for I/O operations
- PDO mapping for I/O data

---

## Layer 4: EtherCAT Manager

### EtherCatManager

**Location:** `myco_ethercat_driver/src/myco_ethercat_manager.cpp`

**Purpose:** Direct interface to SOEM library with real-time cyclic communication.

**Class Definition:**
```cpp
class EtherCatManager {
private:
    const std::string ifname_;          // Network interface name
    uint8_t iomap_[4096];              // Process data image
    int num_clients_;
    boost::thread cycle_thread_;        // Real-time cycle thread
    mutable boost::mutex iomap_mutex_;  // Thread safety
    bool stop_flag_;

public:
    EtherCatManager(const std::string& ifname);
    ~EtherCatManager();
    
    // Process data access
    void write(int slave_no, uint8_t channel, uint8_t value);
    uint8_t readInput(int slave_no, uint8_t channel) const;
    uint8_t readOutput(int slave_no, uint8_t channel) const;
    
    // SDO access
    template <typename T>
    uint8_t writeSDO(int slave_no, uint16_t index, 
                     uint8_t subidx, T value) const;
    template <typename T>
    T readSDO(int slave_no, uint16_t index, uint8_t subidx) const;
    
    int getNumClinets() const;

private:
    bool initSoem(const std::string& ifname);
};
```

**Real-time Cyclic Worker Thread:**

```cpp
void cycleWorker(boost::mutex& mutex, bool& stop_flag) {
    const double period = 1000000; // 1ms in nanoseconds
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);
    
    while (!stop_flag) {
        int expected_wkc = (ec_group[0].outputsWKC * 2) 
                         + ec_group[0].inputsWKC;
        int sent, wkc;
        
        {
            boost::mutex::scoped_lock lock(mutex);
            sent = ec_send_processdata();      // Send RxPDO
            wkc = ec_receive_processdata(EC_TIMEOUTRET); // Receive TxPDO
        }
        
        if (wkc < expected_wkc) {
            handleErrors();  // State machine recovery
        }
        
        // Check for cycle overrun
        clock_gettime(CLOCK_REALTIME, &before);
        if (overrun detected)
            adjust timing;
        
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
        timespecInc(tick, period);  // Next cycle time
    }
}
```

**Initialization Sequence:**

```cpp
bool EtherCatManager::initSoem(const std::string& ifname) {
    // 1. Initialize EtherCAT on network interface
    if (!ec_init(ifname)) return false;
    
    // 2. Auto-discover and configure slaves
    if (ec_config_init(FALSE) <= 0) return false;
    printf("SOEM found %d slaves\n", ec_slavecount);
    
    // 3. Transition to PRE_OP state
    if (ec_statecheck(0, EC_STATE_PRE_OP, timeout) != EC_STATE_PRE_OP)
        return false;
    
    // 4. Configure IO Map (PDO mapping)
    int iomap_size = ec_config_map(iomap_);
    printf("IOMap size: %d\n", iomap_size);
    
    // 5. Configure Distributed Clocks
    ec_configdc();
    
    // 6. Transition to SAFE_OP state
    if (ec_statecheck(0, EC_STATE_SAFE_OP, timeout) != EC_STATE_SAFE_OP)
        return false;
    
    // 7. Transition each slave to OPERATIONAL
    for (int i = 1; i < 5; i++) {
        ec_slave[i].state = EC_STATE_OPERATIONAL;
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_writestate(i);
        
        // Wait up to 40 cycles for operational state
        int chk = 40;
        do {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(i, EC_STATE_OPERATIONAL, 50000);
        } while (chk-- && (ec_slave[i].state != EC_STATE_OPERATIONAL));
        
        if (ec_statecheck(i, EC_STATE_OPERATIONAL, timeout) 
            != EC_STATE_OPERATIONAL)
            return false;
    }
    
    return true;
}
```

**Error Handling:**

```cpp
void handleErrors() {
    ec_group[0].docheckstate = FALSE;
    ec_readstate();
    
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        if ((ec_slave[slave].group == 0) && 
            (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
            
            if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                // Acknowledge error
                ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                ec_writestate(slave);
            }
            else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
                // Return to operational
                ec_slave[slave].state = EC_STATE_OPERATIONAL;
                ec_writestate(slave);
            }
            else if (ec_slave[slave].state > 0) {
                // Reconfigure slave
                if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
                    ec_slave[slave].islost = FALSE;
                    printf("Slave %d reconfigured\n", slave);
                }
            }
            else if (!ec_slave[slave].islost) {
                // Mark as lost
                ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                if (!ec_slave[slave].state) {
                    ec_slave[slave].islost = TRUE;
                    fprintf(stderr, "Slave %d lost\n", slave);
                }
            }
        }
        
        // Recovery for lost slaves
        if (ec_slave[slave].islost) {
            if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
                ec_slave[slave].islost = FALSE;
                printf("Slave %d recovered\n", slave);
            }
        }
    }
}
```

---

## Layer 5: SOEM Library

### Simple Open EtherCAT Master

**Location:** `soem_ros2/SOEM/`

**Purpose:** Open-source EtherCAT master implementation in C.

**Core Modules:**

1. **ethercatmain.c/h:**
   - Main EtherCAT functions
   - `ec_init()`: Initialize master
   - `ec_config_init()`: Configure slaves
   - `ec_config_map()`: Map PDOs
   - `ec_send_processdata()`: Send cyclic data
   - `ec_receive_processdata()`: Receive cyclic data
   - `ec_readstate()`: Read slave states
   - `ec_writestate()`: Write slave states
   - `ec_statecheck()`: Check state transitions

2. **ethercatbase.c/h:**
   - Low-level EtherCAT frame handling
   - Datagram construction
   - Frame transmission/reception
   - Working Counter (WKC) processing

3. **ethercatcoe.c/h:**
   - CANopen over EtherCAT
   - SDO (Service Data Object) read/write
   - SDO info services
   - CoE mailbox communication
   - `ec_SDOread()`: Read object dictionary
   - `ec_SDOwrite()`: Write object dictionary

4. **ethercatconfig.c/h:**
   - Slave configuration
   - EEPROM reading
   - FMMU (Fieldbus Memory Management Unit) setup
   - SM (SyncManager) configuration
   - PDO mapping configuration

5. **ethercatdc.c/h:**
   - Distributed Clock (DC) support
   - Time synchronization
   - `ec_configdc()`: Configure DC
   - DC SYNC0/SYNC1 configuration

6. **ethercatprint.c/h:**
   - Diagnostic output
   - State printing
   - Error code interpretation

7. **nicdrv.c/h (OS-specific):**
   - Network Interface Card driver
   - Raw socket access (Linux)
   - WinPcap/Npcap (Windows)
   - Packet transmission/reception
   - `ecx_setupnic()`: Setup network interface
   - `ecx_send()`: Send Ethernet frame
   - `ecx_recv()`: Receive Ethernet frame

**Key Data Structures:**

```c
// Slave configuration
typedef struct {
    uint16 state;           // Requested state
    uint16 ALstatuscode;    // AL status code
    uint16 configadr;       // Configured station address
    uint16 aliasadr;        // Alias address
    uint32 eep_man;         // Manufacturer ID
    uint32 eep_id;          // Product code
    uint32 eep_rev;         // Revision number
    
    int Obits;              // Output bits
    uint32 Obytes;          // Output bytes
    uint8* outputs;         // Pointer to output process data
    
    int Ibits;              // Input bits
    uint32 Ibytes;          // Input bytes
    uint8* inputs;          // Pointer to input process data
    
    boolean islost;         // Slave lost flag
    // ... many more fields
} ec_slavet;

// Global slave array
extern ec_slavet ec_slave[EC_MAXSLAVE];
extern int ec_slavecount;
```

**EtherCAT State Machine:**

```
        +-------+
        | INIT  |
        +-------+
            |
            v
        +-------+
        |PRE-OP |  <-- Configuration phase
        +-------+
            |
            v
        +-------+
        |SAFE-OP|  <-- PDOs mapped, inputs valid
        +-------+
            |
            v
        +-------+
        |  OP   |  <-- Normal operation, outputs enabled
        +-------+
```

---

## Layer 6: Network Layer

### EtherCAT Protocol

**Protocol Characteristics:**
- **EtherType:** 0x88A4 (EtherCAT)
- **Layer:** OSI Layer 2 (Data Link)
- **Topology:** Line, tree, or star (with switch support)
- **Speed:** 100 Mbps Ethernet
- **Frame size:** 64-1518 bytes (standard Ethernet)

**Frame Structure:**

```
+----------------+
| Ethernet Header|  14 bytes
+----------------+
| EtherCAT Header|  2 bytes (length, type)
+----------------+
| Datagram 1     |  Variable
|   - Command    |    1 byte (Read, Write, RW, etc.)
|   - Index      |    1 byte
|   - Address    |    4 bytes (physical or logical)
|   - Length     |    2 bytes
|   - IRQ        |    2 bytes
|   - Data       |    Variable
|   - WKC        |    2 bytes (Working Counter)
+----------------+
| Datagram 2     |  Variable (optional)
+----------------+
| ...            |
+----------------+
| Ethernet FCS   |  4 bytes (Frame Check Sequence)
+----------------+
```

**Key Concepts:**

1. **On-the-fly Processing:**
   - Slaves process frames as they pass through
   - Data read/written without frame regeneration
   - Extremely low latency (<1 µs per slave)

2. **Working Counter (WKC):**
   - Incremented by each slave processing the datagram
   - Master checks WKC to verify all slaves responded
   - Mismatch indicates communication error

3. **Logical Addressing:**
   - PDO data typically uses logical addressing
   - Master maps logical addresses to slave memory
   - Enables efficient multi-slave read/write in single datagram

4. **Distributed Clocks (DC):**
   - Synchronizes all slave clocks
   - Reference clock: first slave with DC support
   - Typical synchronization accuracy: < 1 µs
   - Important for coordinated motion control

**Communication Cycle:**

```
Master → [Slave 1] → [Slave 2] → [Slave 3] → ... → [Last Slave]
         ↓          ↓          ↓                   ↓
         Data R/W   Data R/W   Data R/W           Data R/W
                                                   ↓
Master ←-----------------------------------------+
(Frame returns to master with all data)
```

**PDO Communication:**
- **Process Data Objects (PDO):** Cyclic, real-time data
- **TxPDO:** Slave to Master (inputs)
- **RxPDO:** Master to Slave (outputs)
- Mapped to slave's process data memory
- Exchanged every cycle (1 ms in this system)

**SDO Communication:**
- **Service Data Objects (SDO):** Acyclic, configuration data
- Mailbox-based communication
- Used for:
  - Parameter configuration
  - Object dictionary access
  - Diagnostics
- Not real-time (can take several ms)

---

## Layer 7: Hardware Layer

### Robot Module Components

**MyCo Robot Configuration:**
- **Typical setup:** 6-axis robot = 3 modules × 2 axes/module
- **EtherCAT slave numbers:** [2, 3, 4] (slave 1 may be I/O or other)

### Motor Drive Units

**Drive Characteristics:**
- **Protocol:** CANopen over EtherCAT (CiA 402)
- **Supported modes:**
  - **CSP (Cyclic Synchronous Position):** Position mode
  - **CSV (Cyclic Synchronous Velocity):** Velocity mode
  - **CST (Cyclic Synchronous Torque):** Torque mode
  - **Homing mode**
  - **Profile position/velocity modes**

**CiA 402 Drive Profile:**
- Standardized state machine
- Standardized object dictionary
- Common control/status words
- Mode-specific parameters

**Safety Features:**
- **STO (Safe Torque Off):** Hardware safety circuit
- **Safe Stop 1/2**
- **Safely Limited Speed/Position**
- Watchdog timers
- Following error monitoring

### Encoders & Sensors

**Encoder Specifications:**
- **Type:** Typically absolute or incremental with battery backup
- **Resolution:** 131072 counts/revolution (axis_position_factor)
- **Interface:** Integrated into drive unit
- **Calibration:** Zero position defined by `count_zero` parameter

**Conversion Formulas:**

```cpp
// Position: Counts → Radians
position_rad = (position_count - count_zero) 
             / (axis_position_factor * reduction_ratio) 
             * 2π;

// Velocity: Counts/cycle → Rad/s
velocity_rad_s = velocity_count 
               / (axis_position_factor * reduction_ratio)
               * 2π / cycle_time;

// Torque: Counts → Nm
torque_Nm = torque_count / axis_torque_factor;
```

**Example Calibration Data:**
```yaml
joint_names: [myco_joint1, myco_joint2, myco_joint3, 
              myco_joint4, myco_joint5, myco_joint6]
reduction_ratios: [121.0, 121.0, 121.0, 121.0, 101.0, 101.0]
count_zeros: [2957134, 9658473, 7019675, 5364919, -363389, -2134627]
axis_position_factors: [131072, 131072, 131072, 131072, 131072, 131072]
axis_torque_factors: [5251.283, 5251.283, 8533.125, 
                      8533.125, 15975.05, 15975.05]
```

### Brakes

**Brake Control:**
- **Type:** Electromagnetic holding brakes
- **Control:** Via EtherCAT commands
- **Services:**
  - `myco_module_open_brake_slaveX`: Release brake (when disabled)
  - `myco_module_close_brake_slaveX`: Engage brake
- **Safety:** Brakes engage automatically on power loss or disable

### I/O Module

**Digital I/O:**
- Separate EtherCAT slave (often slave 5)
- Digital inputs (DI): Read sensor states
- Digital outputs (DO): Control external devices
- Typical I/O: 16-32 DI, 16-32 DO

**Services:**
- `/write_do`: Set output state
- `/read_di`: Query input state
- `/get_io_txpdo`: Read all input PDO
- `/get_io_rxpdo`: Read all output PDO

### Network Infrastructure

**Ethernet Connection:**
- **Interface:** Standard RJ45 Ethernet
- **Cable:** Cat5e or better, shielded recommended
- **Length:** Up to 100m between segments
- **Master PC:** Dedicated EtherCAT network interface (e.g., eth0, ens33)

**Hardware Requirements:**
- **Master PC:**
  - Linux with PREEMPT_RT kernel (recommended)
  - Network interface with raw socket support
  - CPU: Multi-core for real-time performance
  - Priority: Real-time thread management

- **EtherCAT Slaves:**
  - ESC (EtherCAT Slave Controller) - typically ET1100, ET1200, or ASIC
  - Two RJ45 ports (IN and OUT) for daisy-chaining
  - Status LEDs (RUN, ERROR, link)

---

## Data Flow

### Complete Command Path (Top to Bottom)

```
┌──────────────────────────────────────────────────────────┐
│ 1. USER COMMAND                                          │
│    MoveIt2: plan.execute()                               │
│    or MycoBasicAPI: /joint_goal topic                    │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 2. TRAJECTORY GENERATION                                 │
│    MoveIt2 Planning Pipeline → JointTrajectory           │
│    Published to /myco_arm_controller/joint_trajectory    │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 3. TRAJECTORY CONTROLLER (250 Hz)                        │
│    JointTrajectoryController                             │
│    - Receives trajectory                                 │
│    - Interpolates at 250 Hz                              │
│    - Writes to CommandInterface                          │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 4. HARDWARE INTERFACE WRITE                              │
│    MycoHWInterface::write()                              │
│    - Reads position_cmd from CommandInterface            │
│    - Converts radians → counts                           │
│    - Calls setAxisPosCnt() for each axis                 │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 5. ETHERCAT CLIENT                                       │
│    MycoEtherCATClient::setAxis1PosCnt(count)             │
│    - Writes to RxPDO memory location                     │
│    - Updates TARGET_POSITION field                       │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 6. ETHERCAT MANAGER                                      │
│    EtherCatManager::write(slave, channel, value)         │
│    - Thread-safe write to iomap_[channel]                │
│    - Updates ec_slave[slave].outputs[channel]            │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 7. SOEM CYCLIC THREAD (1000 Hz)                          │
│    cycleWorker() loop                                    │
│    - ec_send_processdata()                               │
│      → Copies iomap to EtherCAT frame                    │
│      → Transmits frame to network                        │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 8. ETHERNET LAYER                                        │
│    Raw socket transmission                               │
│    - EtherCAT frame (EtherType 0x88A4)                   │
│    - Sent via network interface (eth0)                   │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 9. PHYSICAL NETWORK                                      │
│    Ethernet cable → First slave → Second slave → ...     │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 10. MOTOR DRIVE (EtherCAT Slave)                         │
│     ESC (EtherCAT Slave Controller)                      │
│     - Extracts TARGET_POSITION from frame                │
│     - Passes to drive control loop                       │
│     - Servo controller processes command                 │
│     - Updates motor current/voltage                      │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 11. SERVO MOTOR                                          │
│     Physical motion executed                             │
└──────────────────────────────────────────────────────────┘
```

### Complete Feedback Path (Bottom to Top)

```
┌──────────────────────────────────────────────────────────┐
│ 1. ENCODER                                               │
│    High-resolution position measurement                  │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 2. MOTOR DRIVE                                           │
│    Servo controller reads encoder                        │
│    - Converts to ACTUAL_POSITION count                   │
│    - Writes to TxPDO memory                              │
│    - Also fills ACTUAL_VELOCITY, ACTUAL_TORQUE,          │
│      STATUSWORD, ERROR_CODE                              │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 3. ETHERCAT SLAVE CONTROLLER                             │
│    ESC inserts TxPDO into returning EtherCAT frame       │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 4. PHYSICAL NETWORK                                      │
│    Frame returns: ... → Slave N → ... → Slave 1 → Master│
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 5. ETHERNET LAYER                                        │
│    Raw socket reception                                  │
│    Network interface receives frame                      │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 6. SOEM CYCLIC THREAD                                    │
│    - ec_receive_processdata()                            │
│      → Extracts data from frame                          │
│      → Writes to iomap                                   │
│      → Updates ec_slave[].inputs[]                       │
│    - Checks Working Counter (WKC)                        │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 7. ETHERCAT MANAGER                                      │
│    EtherCatManager::readInput(slave, channel)            │
│    - Thread-safe read from ec_slave[].inputs[]           │
│    - Returns to caller                                   │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 8. ETHERCAT CLIENT                                       │
│    MycoEtherCATClient::getAxis1PosCnt()                  │
│    - Reads from TxPDO memory location                    │
│    - Returns position count                              │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 9. HARDWARE INTERFACE READ                               │
│    MycoHWInterface::read()                               │
│    - Calls getActPosCounts() for each module             │
│    - Converts counts → radians                           │
│    - Updates StateInterface (position, velocity, effort) │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 10. TRAJECTORY CONTROLLER                                │
│     Reads from StateInterface                            │
│     - Monitors trajectory tracking                       │
│     - Publishes controller state                         │
│     - Checks completion criteria                         │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│ 11. JOINT STATE BROADCASTER                              │
│     Publishes /joint_states topic                        │
│     Used by:                                             │
│     - RViz visualization                                 │
│     - MoveIt2 current state                              │
│     - User applications                                  │
└──────────────────────────────────────────────────────────┘
```

### Timing Diagram

```
Time     EtherCAT (1kHz)    ROS2 Control (250Hz)    MoveIt2 (50Hz)
─────────────────────────────────────────────────────────────────────
0 ms     ├─ Cycle 1         ├─ read()
1 ms     ├─ Cycle 2         │  write()
2 ms     ├─ Cycle 3         │
3 ms     ├─ Cycle 4         │
4 ms     ├─ Cycle 5         ├─ read()                ├─ plan update
5 ms     ├─ Cycle 6         │  write()
6 ms     ├─ Cycle 7         │
7 ms     ├─ Cycle 8         │
8 ms     ├─ Cycle 9         ├─ read()
...      ...                ...                      ...
```

---

## Configuration & Calibration

### Configuration Files

**1. Robot Configuration (`myco_arm_control.yaml`):**

```yaml
/**:
    ros__parameters:
        # Network configuration
        myco_ethernet_name: ens33          # Network interface
        
        # EtherCAT slave configuration
        slave_no: [2, 3, 4]                 # Module slave addresses
        
        # Joint configuration
        joint_names: [myco_joint1, myco_joint2, myco_joint3,
                      myco_joint4, myco_joint5, myco_joint6]
        
        # Mechanical calibration
        reduction_ratios: [121.0, 121.0, 121.0,   # Joints 1-3
                          121.0, 101.0, 101.0]     # Joints 4-6
        
        # Zero position calibration (encoder counts at zero)
        count_zeros: [2957134, 9658473, 7019675,
                     5364919, -363389, -2134627]
        
        # Encoder resolution (counts per encoder revolution)
        axis_position_factors: [131072.0, 131072.0, 131072.0,
                               131072.0, 131072.0, 131072.0]
        
        # Torque scaling (counts per Nm)
        axis_torque_factors: [5251.283, 5251.283, 8533.125,
                             8533.125, 15975.05, 15975.05]

# Controller Manager configuration
controller_manager:
  ros__parameters:
    update_rate: 250                        # Control loop frequency (Hz)
    
    myco_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      joints: [myco_joint1, myco_joint2, myco_joint3,
               myco_joint4, myco_joint5, myco_joint6]
      publish_rate: 50
      write_op_modes: [myco_joint1, myco_joint2, myco_joint3,
                       myco_joint4, myco_joint5, myco_joint6]
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

# Joint Trajectory Controller parameters
myco_arm_controller:
  ros__parameters:
    joints: [myco_joint1, myco_joint2, myco_joint3,
             myco_joint4, myco_joint5, myco_joint6]
    
    command_interfaces: [position, velocity]
    state_interfaces: [position, velocity]
    
    state_publish_rate: 250.0
    action_monitor_rate: 250.0
    allow_partial_joints_goal: false
    
    # Trajectory constraints
    constraints:
      stopped_velocity_tolerance: 0.2       # Rad/s
      goal_time: 0.6                        # Seconds
      
      # Per-joint tolerances
      myco_joint1: { trajectory: 0.2, goal: 0.1 }
      myco_joint2: { trajectory: 0.2, goal: 0.1 }
      myco_joint3: { trajectory: 0.2, goal: 0.1 }
      myco_joint4: { trajectory: 0.2, goal: 0.1 }
      myco_joint5: { trajectory: 0.2, goal: 0.1 }
      myco_joint6: { trajectory: 0.2, goal: 0.1 }
```

**2. Robot-specific Driver Configuration (`myco_drivers.yaml`):**

```yaml
/**:
    ros__parameters:
        myco_driver_name: myco_arm
        
        # Per-module configuration
        # Module 1 (Slave 2):
        myco_client_1_slave_no: 2
        myco_client_1_joint_1_name: myco_joint2
        myco_client_1_joint_2_name: myco_joint1
        myco_client_1_joint_1_reduction_ratio: 121.0
        myco_client_1_joint_2_reduction_ratio: 121.0
        myco_client_1_joint_1_count_zero: 9658473
        myco_client_1_joint_2_count_zero: 2957134
        # ... additional parameters
        
        # Module 2 (Slave 3):
        myco_client_2_slave_no: 3
        # ... similar configuration
        
        # Module 3 (Slave 4):
        myco_client_3_slave_no: 4
        # ... similar configuration
        
        # I/O Module (Slave 5):
        myco_io_client_1_slave_no: 5
```

### Calibration Procedures

**1. Initial Setup:**

```bash
# 1. Connect robot to dedicated network interface
# 2. Verify interface name
ifconfig
# or
ip link show

# 3. Update configuration
nano ~/myco_ws/src/MyCo-ROS2/myco_robot_bringup/config/myco_arm_control.yaml
# Change myco_ethernet_name to match your interface
```

**2. Position Recognition (Zero Calibration):**

```bash
# After mechanical homing, record encoder positions
ros2 service call /get_current_position std_srvs/srv/SetBool "{data: true}"

# Update count_zeros in configuration with reported values
```

**3. Gear Ratio Verification:**

```python
# Verify reduction ratio
# Move joint through known angle and verify encoder counts
expected_counts = angle_rad * (encoder_resolution / (2*pi)) * reduction_ratio
```

**4. Torque Calibration:**

```python
# Measure actual torque with torque sensor
# Compare to reported value
# Adjust axis_torque_factors if needed
actual_torque_Nm = reported_count / axis_torque_factor
```

---

## Real-Time Considerations

### Linux Real-Time Configuration

**1. PREEMPT_RT Kernel:**

```bash
# Install RT kernel
sudo apt-get install linux-image-rt-amd64

# Verify RT kernel
uname -a
# Should show "PREEMPT RT" or "PREEMPT_RT"

# Check RT capabilities
cat /sys/kernel/realtime
# Should return "1"
```

**2. Thread Priority Configuration:**

```cpp
// In EtherCatManager cyclic thread
#include <sched.h>
#include <pthread.h>

void setThreadPriority() {
    struct sched_param param;
    param.sched_priority = 99;  // Highest priority
    
    if (pthread_setschedparam(pthread_self(), 
                              SCHED_FIFO, &param) != 0) {
        fprintf(stderr, "Failed to set thread priority\n");
    }
}
```

**3. CPU Isolation:**

```bash
# Edit /etc/default/grub
GRUB_CMDLINE_LINUX="isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3"

# Update grub
sudo update-grub
sudo reboot

# Pin EtherCAT thread to isolated CPU
taskset -c 2 ./ethercat_node
```

**4. Network Interface Optimization:**

```bash
# Disable power management
sudo ethtool -s eth0 wol d

# Increase ring buffer
sudo ethtool -G eth0 rx 4096 tx 4096

# Disable interrupt coalescing
sudo ethtool -C eth0 rx-usecs 0

# Set static IRQ affinity
echo 2 | sudo tee /proc/irq/<IRQ_NUM>/smp_affinity_list
```

### Timing Analysis

**Update Frequencies:**

| Component | Frequency | Period | Priority |
|-----------|-----------|--------|----------|
| EtherCAT cycle | 1000 Hz | 1 ms | Highest (99) |
| ROS2 control | 250 Hz | 4 ms | High (80) |
| MoveIt2 planning | 50 Hz | 20 ms | Medium (50) |
| Joint state pub | 50 Hz | 20 ms | Medium (50) |
| Diagnostics | 10 Hz | 100 ms | Low (20) |

**Worst-Case Execution Time (WCET):**

```
EtherCAT cycle: < 200 µs
  - ec_send_processdata(): ~50 µs
  - ec_receive_processdata(): ~100 µs
  - handleErrors(): ~50 µs (if needed)

ROS2 control cycle: < 500 µs
  - read(): ~100 µs
  - controller update: ~200 µs
  - write(): ~100 µs

Total latency (command to motor): ~1.5 ms
```

**Jitter Measurements:**

```bash
# Use cyclictest to measure scheduling jitter
sudo cyclictest -p 99 -t1 -n -i 1000 -l 100000

# Target: Max latency < 100 µs
```

### Safety & Watchdogs

**1. EtherCAT Watchdog:**
- Slaves monitor communication timeout
- Configurable timeout (typically 100-500 ms)
- Triggers safe state (STO) on timeout

**2. Controller Watchdog:**
```cpp
// In MycoHWInterface
const double WATCHDOG_TIMEOUT = 0.5;  // 500 ms
rclcpp::Time last_command_time_;

return_type read(...) {
    if ((now() - last_command_time_).seconds() > WATCHDOG_TIMEOUT) {
        // Trigger safe stop
        emergency_stop();
    }
}
```

**3. Following Error Monitoring:**
```cpp
// Check position tracking error
double following_error = fabs(position_cmd - position_actual);
if (following_error > MAX_FOLLOWING_ERROR) {
    trigger_fault();
}
```

**4. Velocity Monitoring:**
```cpp
// Detect unexpected motion
if (!enabled && fabs(velocity) > MOTION_THRESHOLD) {
    trigger_alarm();
}
```

### Fault Recovery

**State Machine Recovery:**

```cpp
void handleErrors() {
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        // Error state → Acknowledge
        if (ec_slave[slave].state == EC_STATE_SAFE_OP + EC_STATE_ERROR) {
            ec_slave[slave].state = EC_STATE_SAFE_OP + EC_STATE_ACK;
            ec_writestate(slave);
        }
        
        // Lost slave → Recover
        if (ec_slave[slave].islost) {
            if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
                ec_slave[slave].islost = FALSE;
                reinitialize_slave(slave);
            }
        }
    }
}
```

**User Recovery Services:**
- `/clear_fault`: Reset drive faults (CiA 402 fault reset)
- `/enable_robot`: Re-enable servos after fault
- `/recognize_position`: Re-home after position loss

---

## System Startup Sequence

### Complete Initialization

**1. Hardware Power-On:**
```
Power supply ON → Motor drives boot → EtherCAT slaves initialize
```

**2. Launch EtherCAT Driver:**
```bash
ros2 launch myco_robot_bringup myco_ros2_ethercat.launch.py
```

**Initialization Steps:**
```
a. EtherCatManager constructor:
   - ec_init(interface_name)
   - Discover slaves (ec_config_init)
   - Configure PDO mapping (ec_config_map)
   - Transition to OPERATIONAL state

b. MycoEtherCATDriver initialization:
   - Load configuration from YAML
   - Create EtherCATClient for each module
   - Initialize ROS2 services and publishers

c. Start cyclic thread:
   - 1 kHz real-time communication loop
   - Continuous PDO exchange
```

**3. Launch ROS2 Control:**
```bash
ros2 launch myco_3_5_950mm_ros2_moveit2 myco_3_5_950mm.launch.py
```

**Steps:**
```
a. Load robot description (URDF)
b. Start controller_manager
c. Initialize MycoHWInterface:
   - Connect to EtherCATDriver
   - Export state/command interfaces
d. Load controllers:
   - joint_state_broadcaster
   - myco_arm_controller
```

**4. Enable Robot:**
```bash
ros2 service call /enable_robot std_srvs/srv/SetBool "{data: true}"
```

**5. Optional: Start MycoBasicAPI:**
```bash
ros2 launch myco_basic_api myco_basic_api.launch.py
```

**6. Ready for Commands:**
```bash
# Via MoveIt2 RViz plugin
# Via topics (/joint_goal, /cart_goal)
# Via action server (FollowJointTrajectory)
```

---

## Troubleshooting Guide

### Common Issues

**1. "No slaves found":**
```bash
# Check network interface
ifconfig

# Verify cable connection
ethtool eth0 | grep "Link detected"

# Check firewall
sudo iptables -L

# Try direct ping (won't work for EtherCAT, but tests link)
# EtherCAT uses Layer 2, not IP
```

**2. "Slave X lost":**
- Check cable integrity
- Verify power supply to slave
- Check EC_TIMEOUTRET value (may need increase)
- Review error logs for slave state

**3. "Working Counter mismatch":**
- Communication error
- PDO configuration mismatch
- Check ESI files match actual hardware
- Verify FMMU/SM configuration

**4. "Following error too large":**
- Commands too aggressive
- Reduce velocity_scaling
- Check trajectory constraints
- Verify mechanical issues (binding, etc.)

**5. Real-time performance issues:**
```bash
# Check for overruns
dmesg | grep -i "overrun"

# Monitor CPU usage
top -H

# Check scheduling latency
sudo cyclictest -p 99 -t1 -n -i 1000
```

### Debug Tools

**1. ROS2 Tools:**
```bash
# List topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Check controller state
ros2 topic echo /myco_arm_controller/state

# Call services
ros2 service call /get_txpdo std_srvs/srv/SetBool "{data: true}"
```

**2. EtherCAT Diagnostics:**
```bash
# View slave information (requires SOEM tools)
# Build simple diagnostic tool using SOEM API

# Check network with Wireshark
sudo wireshark -i eth0 -f "ether proto 0x88a4"
```

**3. System Monitoring:**
```bash
# Real-time performance
sudo trace-cmd record -e sched_switch -e irq

# Network statistics
netstat -i
ethtool -S eth0
```

---

## References & Resources

### Documentation
- **SOEM:** https://github.com/OpenEtherCATsociety/SOEM
- **EtherCAT Technology Group:** https://www.ethercat.org/
- **ROS2 Control:** https://control.ros.org/
- **MoveIt2:** https://moveit.ai/

### Standards
- **IEC 61158:** EtherCAT protocol specification
- **CiA 402:** CANopen device profile for drives
- **IEC 61800-7-201:** Safety functions for drive systems

### Configuration Files Location
```
MyCo-ROS2/
├── myco_robot_bringup/config/
│   ├── myco_arm_control.yaml      # Main configuration
│   └── myco_drivers.yaml           # Driver-specific config
├── myco_*_ros2_moveit2/config/    # MoveIt2 configuration
└── myco_*_ros2_gazebo/config/     # Gazebo simulation config
```

### Key Source Files
```
MyCo-ROS2/
├── myco_ethercat_driver/
│   ├── include/myco_ethercat_driver/
│   │   ├── myco_ethercat_manager.h
│   │   ├── myco_ethercat_driver.h
│   │   └── myco_ethercat_client.h
│   └── src/
│       ├── myco_ethercat_manager.cpp
│       ├── myco_ethercat_driver.cpp
│       └── myco_ethercat_client.cpp
├── myco_ros_control/
│   ├── include/myco_ros_control/
│   │   └── myco_hw_interface.h
│   └── src/
│       └── myco_hardware_interface.cpp
├── myco_basic_api/
│   ├── include/myco_basic_api/
│   │   ├── myco_basic_api.h
│   │   ├── myco_motion_api.h
│   │   └── myco_teleop_api.h
│   └── src/
└── soem_ros2/SOEM/                # SOEM library
```

---

**Document Version:** 1.0  
**Date:** March 10, 2026  
**Author:** Architecture Documentation  
**Target Platform:** Ubuntu 22.04 + ROS2 Humble + PREEMPT_RT

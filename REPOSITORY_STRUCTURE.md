# MyCo-ROS2 — Complete Repository Structure

---

## Top Level

```
MyCo-ROS2/
├── LICENSE                        BSD-style license
├── README.md                      Project overview and setup instructions
└── docs/                          Documentation (API + MoveIt plugin tutorials, images)
    ├── API_description.md
    ├── API_description_english.md
    ├── moveit_plugin_tutorial.md
    ├── moveit_plugin_tutorial_english.md
    └── images/
        ├── execution.png
        ├── MyCo-3.5-0.95.webp
        ├── myco_ethercat_slaves.png
        ├── myco.png
        ├── set_goal_state.png
        ├── set_start_state.png
        └── trajectory.png
```

---

## ROS2 Packages (12 packages total)

---

### 1. `myco_robot_msgs/` — Custom ROS2 Message/Service Definitions

```
myco_robot_msgs/
├── CMakeLists.txt
├── package.xml
└── srv/
    ├── MycoIODRead.srv      Digital I/O read request  (req: bool data → resp: int32 digital_input)
    ├── MycoIODWrite.srv     Digital I/O write request
    ├── SetFloat64.srv       Generic float64 setter service
    ├── SetInt16.srv         Generic int16 setter service
    └── SetString.srv        Generic string setter service
```

---

### 2. `soem_ros2/` — EtherCAT Library (SOEM) wrapped for ROS2

```
soem_ros2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── cmake/
│   └── soem-ros-extras.cmake.in         CMake extras for downstream packages
├── include/soem_ros2/
│   └── soem.h                           Top-level include forwarding to upstream SOEM
└── SOEM/                                Upstream Simple Open EtherCAT Master library
    ├── CMakeLists.txt
    ├── README.md
    ├── soem/                            Core EtherCAT protocol stack
    │   ├── ethercat.h                   Master public API header
    │   ├── ethercatbase.[c/h]           Frame/PDU base layer
    │   ├── ethercatcoe.[c/h]            CANopen over EtherCAT (CoE)
    │   ├── ethercatconfig.[c/h]         Slave auto-configuration
    │   ├── ethercatconfiglist.h         Config list definitions
    │   ├── ethercatdc.[c/h]             Distributed clocks
    │   ├── ethercateoe.[c/h]            Ethernet over EtherCAT (EoE)
    │   ├── ethercatfoe.[c/h]            File over EtherCAT (FoE)
    │   ├── ethercatmain.[c/h]           Master state machine & main loop
    │   ├── ethercatprint.[c/h]          Debug printing utilities
    │   ├── ethercatsoe.[c/h]            Servo over EtherCAT (SoE)
    │   └── ethercattype.h               EtherCAT type definitions
    ├── osal/                            OS abstraction layer
    │   ├── osal.h                       Common OSAL interface
    │   └── <platform>/osal.[c/h]        Per-platform timing & thread primitives
    │       (linux, win32, macosx, rtems, rtk, vxworks, erika, intime)
    ├── oshw/                            OS/HW NIC driver layer
    │   └── <platform>/
    │       ├── nicdrv.[c/h]             NIC raw frame send/receive
    │       └── oshw.[c/h]              HW address utilities
    │       (linux, win32, macosx, rtems, rtk, vxworks, erika, intime)
    ├── cmake/
    │   ├── Modules/Platform/            CMake platform detection files (rtems, rt-kernel)
    │   └── Toolchains/                  Cross-compile toolchain files
    ├── doc/                             Doxygen documentation sources
    └── test/                            Example/test programs
        ├── linux/
        │   ├── simple_test/simple_test.c   Basic master/slave test
        │   ├── slaveinfo/slaveinfo.c        Print slave info
        │   ├── eepromtool/eepromtool.c      EEPROM read/write tool
        │   ├── eoe_test/eoe_test.c          EoE test
        │   ├── aliastool.c                  Alias address tool
        │   ├── ebox/ebox.c                  e-box demo
        │   ├── firm_update/firm_update.c    Firmware update tool
        │   └── red_test/red_test.c          Redundancy test
        ├── win32/                           Windows versions of the above tests
        ├── intime/ec_master/ec_master.c     INtime RTOS test
        └── rtk/main.c                       rt-kernel test
```

---

### 3. `myco_ethercat_driver/` — MyCo EtherCAT Hardware Driver

```
myco_ethercat_driver/
├── CMakeLists.txt
├── package.xml
├── include/myco_ethercat_driver/
│   ├── myco_ethercat_manager.h       Manages EtherCAT bus lifecycle (init/cyclic/shutdown)
│   ├── myco_ethercat_client.h        Per-joint EtherCAT slave client (PDO read/write, enable/fault)
│   ├── myco_ethercat_io_client.h     EtherCAT I/O slave client (digital I/O operations)
│   └── myco_ethercat_driver.h        High-level driver: aggregates clients, exposes ROS2 services
│                                      Services: getTxPDO, getRxPDO, getCurrentPosition,
│                                                getMotionState, enableRobot, disableRobot,
│                                                clearFault, recognizePosition
└── src/
    ├── myco_ethercat_manager.cpp       Bus init, cyclic task, slave state machine
    ├── myco_ethercat_client.cpp        Joint PDO packing/unpacking, enable/disable/clearFault
    ├── myco_ethercat_io_client.cpp     I/O PDO handling
    ├── myco_ethercat_driver.cpp        Driver: reads YAML config, wires clients to ROS2 services
    └── myco_ethercat_driver_node.cpp   ROS2 node entry point for the EtherCAT driver
```

---

### 4. `myco_ros_control/` — ros2_control Hardware Interface Plugin

```
myco_ros_control/
├── CMakeLists.txt
├── package.xml
├── myco_hardware_interface.xml         Plugin XML (registers hardware interfaces with pluginlib)
├── include/myco_ros_control/
│   ├── myco_hw_interface.h             Full hardware interface (position + velocity + effort)
│   │                                    Implements: on_init, on_activate, on_deactivate, read, write
│   ├── myco_hw_onlyposition_interface.h Position-only hardware interface variant
│   └── visibility_control.h            DLL export/import macros
└── src/
    ├── myco_hardware_interface.cpp               Full interface lifecycle callbacks
    └── myco_hardware_onlyposition_interface.cpp  Position-only variant
```

---

### 5. `myco_description/` — Shared URDF & 3D Mesh Robot Descriptions

```
myco_description/
├── CMakeLists.txt
├── package.xml
├── urdf_ros2.rviz                  RViz2 config for visualizing the robot description
├── launch/
│   └── display.launch.py           Launches robot_state_publisher + RViz2
├── urdf/
│   ├── materials.xacro             Shared material color definitions
│   ├── myco_robot.gazebo           Gazebo plugin macros (joint state publisher, ros_control)
│   ├── myco_transmission.xacro    Shared ros2_control transmission macros
│   ├── MyCo-3-0.59.urdf.xacro     URDF for MyCo-3  (590 mm reach, 3 kg payload)
│   ├── MyCo-3.5-0.95.urdf.xacro   URDF for MyCo-3.5 (950 mm reach, 3.5 kg payload)
│   ├── MyCo-5-0.80.urdf.xacro     URDF for MyCo-5  (800 mm reach, 5 kg payload)
│   ├── MyCo-8-1.30.urdf.xacro     URDF for MyCo-8  (1300 mm reach, 8 kg payload)
│   ├── MyCo-10-1.00.urdf.xacro    URDF for MyCo-10 (1000 mm reach, 10 kg payload)
│   └── MyCo-15-1.30.urdf.xacro    URDF for MyCo-15 (1300 mm reach, 15 kg payload)
└── meshes/                         STL collision/visual meshes per robot variant
    ├── MyCo-3-0.59/
    │   ├── myco_base.STL
    │   ├── myco_link1.STL
    │   ├── myco_link2.STL
    │   ├── myco_link3.STL
    │   ├── myco_link4.STL
    │   ├── myco_link5.STL
    │   ├── myco_link6.STL
    │   └── myco_end_link.STL
    ├── MyCo-3.5-0.95/   (same set of 8 STL files)
    ├── MyCo-5-0.80/     (same set of 8 STL files)
    ├── MyCo-8-1.30/     (same set of 8 STL files)
    ├── MyCo-10-1.00/    (same set of 8 STL files)
    └── MyCo-15-1.30/    (same set of 8 STL files)
```

---

### 6. `myco_basic_api/` — High-Level Motion API (MoveIt2-based)

```
myco_basic_api/
├── CMakeLists.txt
├── package.xml
├── cfg/
│   └── MyCoBasicAPIDynamicReconfigure.cfg   Dynamic reconfigure parameter definitions
│                                              (velocity scaling, etc.)
├── include/myco_basic_api/
│   ├── myco_basic_api_const.h        Shared constants (tolerances, default frame names)
│   ├── myco_basic_api_node.h         Node-level includes and forward declarations
│   ├── myco_basic_api.h              Top-level MycoBasicAPI class
│   │                                  Composes MycoTeleopAPI + MycoMotionAPI
│   │                                  Services: setRefLink, setEndLink,
│   │                                            enableRobot, disableRobot,
│   │                                            setVelocityScaling, updateVelocityScaling
│   │                                  Subscriber: /set_vel (Float32)
│   │                                  Dynamic reconfig: velocity scaling callback
│   ├── myco_motion_api.h             MoveIt2 motion planning class
│   │                                  Subscribers: joint_goal (JointState),
│   │                                               cart_goal (PoseStamped),
│   │                                               cart_path_goal (PoseArray)
│   │                                  Services: getRefLink, getEndLink
│   │                                  Publisher: joint torques (Float64MultiArray)
│   │                                  Methods: trajectoryScaling, setVelocityScaling, setRefFrames
│   └── myco_teleop_api.h             Teleoperation / incremental jogging class
│                                      Services: step in joint space, step in cartesian space
├── src/
│   ├── myco_basic_api_node.cpp       Main entry point
│   │                                  - Creates two nodes: move_group node + base_api node
│   │                                  - Initialises MoveGroupInterface (group: myco_arm)
│   │                                  - Calls startStateMonitor + getCurrentJointValues
│   │                                  - Creates RobotModelLoader + PlanningSceneMonitor
│   │                                  - Starts scene publishing at 25 Hz
│   │                                  - Instantiates MycoBasicAPI
│   │                                  - Spins base_api_node
│   ├── myco_basic_api.cpp            MycoBasicAPI implementation
│   │                                  Controller switching (stop active, start myco controller)
│   ├── myco_motion_api.cpp           Motion planning & execution via MoveIt2 action client
│   └── myco_teleop_api.cpp           Incremental jog implementation
├── launch/
│   ├── myco_basic_api.launch.py      Launches the basic API node (real robot)
│   ├── myco_gui.launch.py            Launches API + PyQt5 GUI (real robot)
│   └── fake_myco_gui.launch.py       Launches API + PyQt5 GUI (fake/simulated robot)
├── scripts/
│   ├── myco_gui.py                   PyQt5 GUI for manual teleoperation and motion control
│   ├── test.py                       Quick integration test script
│   └── btn_icon/                     Button icon PNG assets for the GUI
│       ├── End_btn0_high.png
│       ├── End_btn0_low.png
│       ├── End_btn1_high.png
│       ├── End_btn1_low.png
│       ├── End_btn2_high.png
│       ├── End_btn2_low.png
│       ├── End_btn3_high.png
│       └── End_btn3_low.png
└── myco_basic_api/
    └── __init__.py                   Python package marker
```

---

### 7. `myco_robot_bringup/` — Real Robot Full Bringup

```
myco_robot_bringup/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── myco_drivers.yaml              EtherCAT driver params (slave IDs, reduction ratios,
│   │                                   position/torque factors, count zeros per joint)
│   ├── myco_arm_control.yaml          Joint trajectory controller config (PID gains, limits)
│   ├── myco_module_control.yaml       Module-level controller config
│   └── joint_state_controller.yaml    Joint state broadcaster config
└── launch/
    └── myco_ros2_ethercat.launch.py   Full bringup:
                                        EtherCAT driver + ros2_control + controllers + state pub
```

---

### 8–13. Robot Variant Packages (one pair per robot model)

Each robot variant has two packages with identical internal structure:

---

#### `myco_<variant>_ros2_gazebo/` — Gazebo Simulation Package

```
myco_<variant>_ros2_gazebo/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── myco_arm_controller.yaml        ros2_control joint trajectory controller for simulation
├── launch/
│   └── myco_<variant>_gazebo.launch.py Spawns robot URDF in Gazebo + loads controllers
├── urdf/
│   ├── MyCo-<variant>.urdf.xacro       Top-level URDF (includes description + gazebo + transmission)
│   ├── myco_<variant>_gazebo.xacro     Gazebo plugin block (physics properties, sensors)
│   └── myco_<variant>_transmission.xacro  ros2_control transmission macros
└── worlds/
    └── myco_<variant>.world            Gazebo world file (ground plane, lighting)
```

#### `myco_<variant>_ros2_moveit2/` — MoveIt2 Configuration Package

```
myco_<variant>_ros2_moveit2/
├── CMakeLists.txt
├── package.xml
├── .setup_assistant                    MoveIt Setup Assistant metadata (SRDF generation config)
├── config/
│   ├── MyCo-<variant>.urdf.xacro       URDF with ros2_control hardware tag (real or fake)
│   ├── MyCo-<variant>.ros2_control.xacro  ros2_control hardware interface block
│   ├── MyCo-<variant>.srdf             Semantic Robot Description:
│   │                                    planning groups, named poses, disabled collisions
│   ├── kinematics.yaml                 IK solver config (KDL or TRAC-IK, position tolerance)
│   ├── joint_limits.yaml               Position, velocity, acceleration limits per joint
│   ├── initial_positions.yaml          Home/zero joint position values
│   ├── ompl_planning.yaml              OMPL planner parameters (RRTConnect, PRM, etc.)
│   ├── pilz_cartesian_limits.yaml      Pilz Industrial Motion planner Cartesian speed limits
│   ├── moveit_controllers.yaml         MoveIt → ros2_control action controller mapping
│   ├── ros2_controllers.yaml           ros2_control controller definitions
│   └── moveit.rviz                     RViz2 MoveIt plugin display config
└── launch/
    ├── myco_<variant>.launch.py             Main launch: real robot + MoveIt2 + RViz2
    ├── myco_<variant>_moveit.launch.py      MoveIt2 move_group only
    ├── myco_<variant>_moveit_rviz.launch.py MoveIt2 + RViz2
    ├── myco_<variant>_basic_api.launch.py   MoveIt2 + myco_basic_api node
    ├── move_group.launch.py                 move_group node with full MoveIt2 capabilities
    ├── moveit_rviz.launch.py                RViz2 with MoveIt plugin
    ├── rsp.launch.py                        Robot State Publisher
    ├── spawn_controllers.launch.py          Spawn and activate ros2_control controllers
    ├── static_virtual_joint_tfs.launch.py   Static TF broadcaster for virtual joints
    ├── demo.launch.py                       Standalone demo with fake hardware (no real robot)
    ├── setup_assistant.launch.py            Launch MoveIt Setup Assistant for reconfiguration
    └── warehouse_db.launch.py               MongoDB warehouse for storing MoveIt motion plans
```

---

#### Robot Variant Summary

| Package prefix       | Robot    | Payload | Reach   |
|----------------------|----------|---------|---------|
| `myco_3_590mm`       | MyCo-3   | 3 kg    | 590 mm  |
| `myco_3_5_950mm`     | MyCo-3.5 | 3.5 kg  | 950 mm  |
| `myco_5_800mm`       | MyCo-5   | 5 kg    | 800 mm  |
| `myco_8_1300mm`      | MyCo-8   | 8 kg    | 1300 mm |
| `myco_10_1000mm`     | MyCo-10  | 10 kg   | 1000 mm |
| `myco_15_1300mm`     | MyCo-15  | 15 kg   | 1300 mm |

---

## Dependency Stack (bottom → top)

```
SOEM (upstream EtherCAT library — soem_ros2/SOEM/)
    └── soem_ros2                   wraps SOEM as a ROS2 colcon package
            └── myco_ethercat_driver    robot-specific EtherCAT slaves & services
                    └── myco_ros_control    ros2_control SystemInterface plugin
                            └── myco_robot_bringup  full real-robot launch

myco_robot_msgs                     custom .srv definitions (used by driver & basic_api)

myco_description                    shared URDF + STL meshes (all 6 variants)
    ├── myco_<variant>_ros2_gazebo      Gazebo simulation (per variant)
    └── myco_<variant>_ros2_moveit2     MoveIt2 config (per variant)
                └── myco_basic_api      high-level MoveIt2 motion/teleop API + PyQt5 GUI
```

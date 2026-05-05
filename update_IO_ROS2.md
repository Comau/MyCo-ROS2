# MyCo ROS2 IO Quick Reference

## Write Digital Outputs (DO)

### All DO down
```bash
ros2 service call /write_do myco_robot_msgs/srv/MycoIODWrite "digital_output: 0"
```

### DO0 up
```bash
ros2 service call /write_do myco_robot_msgs/srv/MycoIODWrite "digital_output: 1"
```

### DO1 up
```bash
ros2 service call /write_do myco_robot_msgs/srv/MycoIODWrite "digital_output: 2"
```

### DO2 up
```bash
ros2 service call /write_do myco_robot_msgs/srv/MycoIODWrite "digital_output: 4"
```

### (DO0 & DO1) up
```bash
ros2 service call /write_do myco_robot_msgs/srv/MycoIODWrite "digital_output: 3"
```

### (DO0 & DO2) up
```bash
ros2 service call /write_do myco_robot_msgs/srv/MycoIODWrite "digital_output: 5"
```

### (DO1 & DO2) up
```bash
ros2 service call /write_do myco_robot_msgs/srv/MycoIODWrite "digital_output: 6"
```

### (DO0 & DO1 & DO2) up
```bash
ros2 service call /write_do myco_robot_msgs/srv/MycoIODWrite "digital_output: 7"
```

---

## Read Digital Inputs (DI)

```bash
ros2 service call /read_di myco_robot_msgs/srv/MycoIODRead "data: true"
```

| `digital_input` | Meaning |
|---|---|
| `0` | DI down |
| `1` | DI0 up |
| `2` | DI1 up |
| `4` | DI2 up |
| `3` | (DI0 & DI1) up |
| `5` | (DI0 & DI2) up |
| `6` | (DI1 & DI2) up |
| `7` | (DI0 & DI1 & DI2) up |

---

## LED Colors

| `digital_output` | Color |
|---|---|
| `0` | None |
| `1` | Green |
| `2` | Red |
| `3` | Yellow |
| `4` | Blue |
| `5` | Cyan |
| `6` | Magenta |
| `7` | White |

Set LED color example:

```bash
ros2 service call /led_color myco_robot_msgs/srv/MycoIODWrite "digital_output: 1"
```

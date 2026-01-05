# 🗺️ MappingBot – IoT-Enabled Indoor Floor Mapping Robot

An **IoT-enabled differential drive robot** for **indoor mapping and localization** using **ROS 2**, **Arduino**, **ultrasonic SLAM**, and **Nav2**.
The robot builds a 2D occupancy map using **wheel odometry + a servo-mounted ultrasonic sensor** and visualizes it in **RViz**.

---

## 🔧 Hardware Requirements

* Arduino UNO
* Raspberry Pi 4B (or Linux PC / WSL2)
* L298N Motor Driver
* DC Motors
* Rotary encoder (I attached the rotary encoders to the wheels to aquire odometry values)
* Ultrasonic Sensor (HC-SR04)
* Servo Motor
* External 12V power supply
---

## 💻 Software Stack

* **OS**: Ubuntu 22.04 (Native / WSL2)
* **ROS 2**: Humble Hawksbill

* **Tools & Packages**:

  * slam_toolbox
  * navigation2 (Nav2)
  * teleop_twist_keyboard
  * tf2_ros
  * rviz2

---

## 📁 Repository Structure

```
MappingBot/
│
├── src/my_bot/          # ROS 2 packages
│   ├── launch/
│   ├── config/
│   ├── description/
│   ├── scripts
│   └── ...
│
├── arduino/             # Arduino firmware
│   └── mapping_bot_firmware.ino

```

---

## 🔌 Arduino Setup

1. Open **Arduino IDE**
2. Upload firmware from:

   ```
   arduino/mapping_bot_firmware.ino
   ```
3. Set baud rate to **115200**
4. Ensure motors, encoders, ultrasonic sensor, and servo are wired correctly
5. Power motors using an **external battery** (do NOT power motors from USB)

---

## 🪟 Windows + WSL2 Setup (IMPORTANT)

If you are using **Windows 10/11 with WSL2**, you must attach the Arduino USB device before running ROS. (I do this step to be sure)

### Step 1: Open PowerShell as Administrator

```powershell
usbipd list
```

Identify your Arduino Bus ID (example: `1-3`), then attach it:

```powershell
usbipd attach --wsl --busid 1-3
```

---

### Step 2: Inside WSL (Ubuntu Terminal)

Check serial devices:

```bash
ls /dev/tty*
```

You should see:

```bash
/dev/ttyACM0
```
or
```bash
/dev/ttyACM1
```

Give permission:

```bash
sudo chmod 666 /dev/ttyACM0
```

---

## 🚀 Build the ROS Workspace

```bash
cd ~/MappingBot
colcon build
source install/setup.bash
```

---

## ▶️ Running the Robot (Recommended Order)

### 1️⃣ Robot Bringup

```bash
ros2 launch my_bot test.launch.py
```

This launches:

* Serial bridge
* Encoder odometry
* Ultrasonic scan publisher
* TF tree

---

### 2️⃣ Start SLAM Toolbox

```bash
ros2 launch slam_toolbox online_async_launch.py \
params_file:=./iot_ws/src/my_bot/config/mapper_params_online_async.yaml \
use_sim_time:=true
```

---

### 3️⃣ Start RViz  in a new terminal

```bash
rviz2
```

In RViz:

* Fixed Frame → `map`
* Add **LaserScan**
* Add **Odometry**
* Add **TF**

---

### 4️⃣ Teleoperate the Robot (In another terminal)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keys present on screen to move the robot and perform mapping.

---

## 🔗 Static Transforms (If Required - You might see errors in rviz to counter them do the following in new terminals)

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
```

---

## 🗺️ Save the Map

After completing mapping:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

This generates:

```
~/maps/my_map.pgm
~/maps/my_map.yaml
```
The map on rviz will look something like below
<img width="1366" height="724" alt="Screenshot 2025-11-06 114830" src="https://github.com/user-attachments/assets/fe1f6a16-5ab0-4117-9a44-930dc98f8956" />

---

## 🧭 Navigation (Nav2) 

Install required packages:

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

Launch navigation:

```bash
ros2 launch my_bot nav2_ultrasonic.launch.py
```

You can now send **2D Nav Goals** from RViz.

---

## ⚠️ Common Issues

| Issue                    | Solution                                 |
| ------------------------ | ---------------------------------------- |
| `/dev/ttyACM0` not found | Attach USB using `usbipd` or check cable |
| Motors not moving        | Ensure external motor power              |
| No scan visible          | Check servo + ultrasonic wiring          |
| TF errors                | Run static TF publishers                 |
| Robot drift              | Calibrate encoders                       |

---

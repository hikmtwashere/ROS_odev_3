# ROS Noetic Engelden Kaçınma ve PID Ödevi

Bu repo, ROS Noetic üzerinde hazırlanmış `odev3` paketini içerir.

## İçerik

- `move_stop_rotate.py`: Robot engel yoksa ileri gider, engel algılarsa durur ve döner.
- `pid_controller.py`: Robotun önündeki engele olan mesafesine göre PID tabanlı hız kontrolü yapar.

## Kullanılan Topicler

| Topic | Mesaj Tipi |
|---|---|
| `/scan` | `sensor_msgs/LaserScan` |
| `/cmd_vel` | `geometry_msgs/Twist` |

## Çalıştırma

Workspace derlenir:

```bash
cd ~/rbtg_ws
catkin_make
source devel/setup.bash

Move-Stop-Rotate çalıştırılır:

rosrun odev3 move_stop_rotate.py

PID kontrolcü çalıştırılır:

rosrun odev3 pid_controller.py

Not: İki script aynı anda çalıştırılmamalıdır. Çünkü ikisi de /cmd_vel topic'ine komut gönderir.

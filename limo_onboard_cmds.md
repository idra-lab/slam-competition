
# LIMO Onboard SLAM Competition Commands

## Network setup

Connect your laptop and the LIMO to your hotspot.

## On the LIMO Robot
Find your IP address: 

```bash
ifconfig
```

read the address under `wlan0`.

## On Your Laptop

Log into the LIMO robot via SSH after finding its IP address.

```bash
ssh agilex@*ip.address*
```
e.g
```bash
ssh agilex@172.20.10.2
```

It will ask for a password, which is: 
`agx`

Then, when asked if you want to select ros1 or ros2, type `2` and hit enter.

## On you SSH terminals in your laptop

First remove this file if it exists:

```bash
rm -rf ~/mapping
```

You will need 5 terminals for the following commands:

1. Terminal 1:

```bash
ros2 launch limo_bringup limo_start.launch.py

```

2. Terminal 2:

```bash
ros2 launch orbbec_camera dabai.launch.py
```

3. Terminal 3:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

4. Terminal 4:

```bash
ros2 launch limo_bringup cartographer.launch.py
```

5. Terminal 5:

```bash
ros2 bag record -a -o mapping
```

## Optional Commands
<!-- ros2 run rqt_image_view rqt_image_view -->

<!-- ros2 launch nav2_bringup navigation_launch.py -->

# Move the Robot around to map the environment

Read the instructions written in terminal 3 and learn to move the car.

# Challenge: 
Complete one complete loop around the table and stop the robot where it started.


## On Your laptop

Once you are done, attach the ethernet cable and copy the mapping folder to your laptop. 

Find again the LIMO robot IP address using `ifconfig` command.

Then, on your laptop, run:

```bash
scp -r agilex@*ip.address*:~/mapping .
```


# Congratulations! You have successfully completed the challenge


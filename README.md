# Kinova Learning from Demonstrations (Kinova-LfD)

A framework for learning from demonstrations developed for the Intelligent and Reliable Autonomous Systems (IRAS) Lab (Dr. Sandhya Saisubramanian) at Oregon State University. 

Primary Developers: Will Solow (soloww@oregonstate.edu)

TODOs:
convert everything onto CPU for faster runtime? and figure out how to get camera info.

## Installation

1. Create a conda environment: `conda create -n kinova-lfd python=3.12`
2. Activate conda environment: `conda activate kinova-lfd`
3. Install requirements via pip: `pip install -e .`

## Kortex Integration
Sometimes ROS2 is a pain. To run the robot through the Kortex API instead of ROS2, follow these directions:
1. `python3 -m pip install kinova_lfd/envs/base/kortex_api-2.6.0.post3-py3-none-any.whl`
2. `pip install -e ".[kortex]"

You should now be all set to run an experiment with `--env_id Kortex-Gen3-v0`


sudo ip addr add 192.168.1.100/24 dev enx<your_usb_adapter>
sudo ip link set enx<your_usb_adapter> up
sudo ip route replace 192.168.1.0/24 dev enx<your_usb_adapter>

sudo ip addr add 192.168.1.100/24 dev enx6c6e072d4846
sudo ip link set enx6c6e072d4846 up
sudo ip route add 192.168.1.0/24 dev enx6c6e072d4846

# Create a persistent static connection profile
sudo nmcli con add \
  type ethernet \
  ifname enx6c6e072d4846 \
  con-name kinova-robot \
  ipv4.method manual \
  ipv4.addresses 192.168.1.100/24 \
  ipv4.never-default yes \
  connection.autoconnect yes
  sudo nmcli con up kinova-robot
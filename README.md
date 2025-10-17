# mocapapi_python

![repo size](https://img.shields.io/github/repo-size/takuya-ki/mocapapi_python)

- Docker environment for Perception Neuron Python interface. This repository was inspired by [pnmocap/mocapapi_python](https://github.com/pnmocap/mocapapi_python).
- Example scripts for Perception Neuron.

## Dependencies

### Docker build environments

- [Ubuntu 22.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=22.04+LTS)
  - Docker 27.4.1
  - Docker Compose 2.32.1

## Installation

1. Connect the USB between the pn studio hub and your Windows computer (if both of the computers are in the same LAN)  
    <img src=image/access_point.jpg width=280>  
2. Set up the communication rules by following these (in case of using Windows defender)
    1. Open wf.msc<br>
    <img src=image/wf.msc.png width=280><br>
    2. Add inbound / outbound rules for 7001 and 7012<br>
    <img src=image/inbound_rule_step1.png width=280><br>
    <img src=image/inbound_rule_step2.png width=280><br>
    <img src=image/inbound_rule_step3.png width=280><br>
    <img src=image/inbound_rule_step4.png width=280><br>
    <img src=image/inbound_rule_step5.png width=280><br>
3. Install Axis Studio downloaded from [here](https://shop.noitom.com.cn/common/product_item/index.jhtml?productItemId=3)
    - Note that if you download from other places, a license may be required to open the software
    - Also, please unbind the old computer you were previously using when installing Axis Studio on your new computer at [here](https://account.noitom.com/#/login)
4. Set up the Axis Studio software by following [this](https://github.com/pnmocap/mocap_ros_py?tab=readme-ov-file#configure-the-motion-capture-software-axis-studio)  
    1. Make sure the network settings are configured as shown below:<br>
    <img src=image/network.png width=280><br>
    2. Enable the connection:<br>
    <img src=image/setting.png width=280><br>
    3. Perform calibration so the system correctly recognizes joint angles:<br>
    <img src=image/calibration.png width=280><br>
5. Set the ip address 100.80.147.8 to your Ubuntu PC
6. Build the Docker environment as shown below (if you are using Docker, this must be done inside the container):  
    ```bash
    nc -zvu 100.80.147.42 7001
    ```
    ```bash
    git clone git@github.com:takuya-ki/mocapapi_python.git --recursive --depth 1 && cd mocapapi_python && COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel  
    ```

## Usage with docker

1. Build and run the docker environment
    - Create and start docker containers in the initially opened terminal
    ```bash
    docker compose up
    ```
2. Turn on the Axis Studio on a Windows machine and start capturing the motions
3. Kill the processes previously executed and execute the bringup launch inside the container
    ```bash
    xhost + && docker exec -it pnmocap_python_container bash
    ```
    ```bash
    python3 mocap_api.py
    ```

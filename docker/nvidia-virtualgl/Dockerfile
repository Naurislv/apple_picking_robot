FROM ros

RUN apt update


# VirtualGL & headless NVIDIA Xorg
RUN apt install -y xserver-xorg-core libxv1 x11-xserver-utils curl

WORKDIR /tmp
RUN curl -L -o virtualgl_2.5.2_amd64.deb 'https://downloads.sourceforge.net/project/virtualgl/2.5.2/virtualgl_2.5.2_amd64.deb?r=https%3A%2F%2Fsourceforge.net%2Fprojects%2Fvirtualgl%2Ffiles%2F2.5.2&ts=1509495317&use_mirror=auto'
RUN dpkg -i virtualgl_2.5.2_amd64.deb
RUN printf "1\nn\nn\nn\nx\n" | /opt/VirtualGL/bin/vglserver_config

# nvidia-xconfig --use-display-device=None --allow-empty-initial-configuration \
#   --virtual=1920x1080 --mode=1920x1080 --no-use-edid-dpi -o xorg.conf.nvidia-headless
COPY xorg.conf.nvidia-headless /etc/X11/xorg.conf

# create startx script for starting X server manually
RUN echo 'echo "Starting X server ..."\n\
Xorg vt10 :0 &\n\
echo "X server log: /var/log/Xorg.0.log"\n\
echo\n' > /usr/local/bin/startx
RUN chmod +x /usr/local/bin/startx

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility



# Install some required and useful tools
RUN apt install -y git curl vim

# Clone repo and install dependencies
WORKDIR /root
RUN git clone https://github.com/Naurislv/apple_picking_robot.git -b virtual_env

WORKDIR /root/apple_picking_robot/ros

RUN /bin/bash -c ' \
    source /opt/ros/$ROS_DISTRO/setup.bash ; \
    ROS_PACKAGE_PATH=$PWD:$ROS_PACKAGE_PATH ; \
    rosdep -y -i install turtlebot3_description turtlebot3_navigation turtlebot3_fake turtlebot3_gazebo && \
    catkin_make \
'

ENV TURTLEBOT3_MODEL burger

COPY entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]

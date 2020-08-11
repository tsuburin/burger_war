FROM osrf/ros:kinetic-desktop-full-xenial AS burger

# gazebo7のアップデート & 依存パッケージインストール
#
# dbusについて
#   ときどきコマンド実行時に/etc/machine-idがないと怒られることがあるので
#   dbus-uuidgenコマンドでそれを生成するのに使用する
#     免責事項：この怒られの原因は理解していないので、この対処法が正しいという保証はないです。
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y wget \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
         python-pip \
         ros-kinetic-turtlebot3 \
         ros-kinetic-turtlebot3-msgs \
         ros-kinetic-turtlebot3-simulations \
         ros-kinetic-aruco-ros \
         gazebo7 \
         dbus \
    && rm -rf /var/lib/apt/lists/*

RUN pip install requests flask

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

ARG UID
ARG GID
RUN groupadd -g ${GID:?} burger && useradd -m -s /bin/bash -u ${UID:?} -g burger burger

RUN echo "burger ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER burger
WORKDIR /home/burger

ENV GAZEBO_MODEL_PATH=/home/burger/catkin_ws/src/burger_war/burger_war/models/
ENV TURTLEBOT3_MODEL=burger
ENV QT_X11_NO_MITSHM=1
ENV CATKIN_WS=/home/burger/catkin_ws
ENV BURGER_WAR_REPO=/home/burger/catkin_ws/src/burger_war

RUN mkdir /home/burger/.gazebo \
    && mkdir -p /home/burger/catkin_ws/src \
    && cd /home/burger/catkin_ws/src \
    && echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc \
    && bash -i -c "catkin_init_workspace" \
    && cd /home/burger/catkin_ws \
    && bash -i -c "catkin_make" \
    && echo "source /home/burger/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && echo 'export PATH=$BURGER_WAR_REPO/docker-compose/bin:$PATH' >> ~/.bashrc

FROM burger as developer

# 開発環境インストール
USER root
RUN apt-get update \
    && apt-get install -y \
      software-properties-common \
      apt-transport-https \
    && add-apt-repository ppa:neovim-ppa/stable \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update \
    && apt-get install -y \
         neovim \
         python3.7 \
    && bash -c "python3.7 <(curl https://bootstrap.pypa.io/get-pip.py)" \
    && pip3.7 install neovim \
    && bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)" llvm.sh 10 \
    && ln -s $(which clangd-10) /usr/local/bin/clangd \
    && rm -rf /var/lib/apt/lists/*

USER burger
WORKDIR /home/burger/catkin_ws
RUN ln -s build/compile_commands.json ./
WORKDIR /home/burger/catkin_ws/src/burger_war

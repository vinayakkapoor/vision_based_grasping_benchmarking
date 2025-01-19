FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get -y install \
    git \
    python3-pip \
    curl \
    libcanberra-gtk-module \
    libcanberra-gtk3-module
RUN apt-get clean

RUN cd ~/
RUN git clone https://github.com/vinayakkapoor/vision_based_grasping_benchmarking.git /repo
WORKDIR /repo
RUN chmod +x ./setup2.sh benchmark_grasping2.sh
CMD ["./setup2.sh"]

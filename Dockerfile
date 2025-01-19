FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get -y install \
    git \
    python3-pip \
    libcanberra-gtk-module \
    libcanberra-gtk3-module
RUN apt-get clean



CMD ["firefox"]
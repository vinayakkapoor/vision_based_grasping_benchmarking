FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV CUDA_VERSION=11.8

RUN apt-get update
RUN apt-get -y install \
    git \
    python3-pip \
    curl \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    wget
RUN apt-get clean

# Install cuda
# RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb && \
#     dpkg -i cuda-keyring_1.0-1_all.deb && \
#     apt-get update && \
#     apt-get install -y cuda-toolkit-11-8 && \
#     rm cuda-keyring_1.0-1_all.deb

# RUN git clone https://github.com/vinayakkapoor/vision_based_grasping_benchmarking.git /root/vision_based_grasping_benchmarking
COPY /grasping_benchmarking_suite /root/vision_based_grasping_benchmarking/grasping_benchmarking_suite/
WORKDIR /root/vision_based_grasping_benchmarking/grasping_benchmarking_suite/
RUN chmod +x ./setup.sh benchmark_grasping.sh
RUN ./setup.sh
WORKDIR /root/grasping_benchmarking
CMD ["./benchmark_grasping.sh"]

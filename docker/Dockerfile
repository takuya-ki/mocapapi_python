# base image
FROM ubuntu:22.04

# setting environmental variables
ENV DEBIAN_FRONTEND noninteractive
ARG MAKEFLAGS
ARG PARALLELWORKERS
ARG DISPLAY
ARG WAYLAND_DISPLAY
ARG XDG_RUNTIME_DIR
ARG PULSE_SERVER
ENV MAKEFLAGS $MAKEFLAGS
ENV PARALLELWORKERS $PARALLELWORKERS
ENV DISPLAY $DISPLAY
ENV WAYLAND_DISPLAY $WAYLAND_DISPLAY
ENV XDG_RUNTIME_DIR $XDG_RUNTIME_DIR
ENV PULSE_SERVER $PULSE_SERVER

# setting time zone
RUN apt update && apt install -y --no-install-recommends tzdata
ENV TZ=Asia/Tokyo

RUN apt update -y && apt upgrade -y && apt install -y --no-install-recommends \
    software-properties-common \
    iputils-ping net-tools \
    libxkbcommon-x11-0 \
    build-essential \
    libgl1-mesa-dev \
    lsb-release \
    libudev-dev \
    python3-dev \
    python3-pip \
    python3-tk \
    mesa-utils \
    net-tools \
    xorg-dev \
    x11-apps \
    libfuse2 \
    libxcb* \
    netcat \
    gnupg \
    byobu \
    cmake \
    curl \
    wget \
    lsof \
    dbus \
    git \
    vim \
    zip \
    && rm -rf /var/lib/apt/lists/*

RUN pip install docutils==0.21.2 pynput==1.8.1

# Generates bash commands
WORKDIR /root/src/
RUN echo "ping 100.80.147.72" >> ~/.bash_history
RUN echo "nc -zvu 100.80.147.72 7001" >> ~/.bash_history
RUN echo "lsof -i UDP:7012" >> ~/.bash_history
RUN echo "python3 mocap_to_stickman.py" >> ~/.bash_history
RUN echo "python3 mocap_api.py" >> ~/.bash_history

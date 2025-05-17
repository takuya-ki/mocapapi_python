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
    texlive-fonts-recommended \
    texlive-latex-extra \
    iputils-ping net-tools \
    libxkbcommon-x11-0 \
    gnome-screenshot \
    libcanberra-gtk* \
    build-essential \
    libgl1-mesa-dev \
    lsb-release \
    libudev-dev \
    imagemagick \
    python3-dev \
    python3-pip \
    python3-tk \
    mesa-utils \
    net-tools \
    aptitude \
    xorg-dev \
    x11-apps \
    libfuse2 \
    cm-super \
    texlive \
    libxcb* \
    dvipng \
    unzip \
    gnupg \
    byobu \
    cmake \
    curl \
    wget \
    dbus \
    git \
    vim \
    zip \
    && rm -rf /var/lib/apt/lists/*

RUN pip install docutils==0.21.2 pynput==1.8.1

# Generates bash commands
WORKDIR /root/src/
RUN echo "python3 mocap_main.py" >> ~/.bash_history

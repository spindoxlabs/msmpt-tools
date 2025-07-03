# FROM osrf/ros:humble-desktop-full
FROM ubuntu:22.04

RUN apt update && apt install -y \
  python3 \
  python3-pip \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY ./src/requirements.txt .
RUN pip3 install -r requirements.txt


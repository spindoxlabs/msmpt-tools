# msmpt-tools

This repository provides the code associated with the paper titled "Dataset and tools for benchmarking Multi-Sensor Multi-People Tracking for Ground Robots" that is currently under revision at Data in Brief.

## Requirements

To run the code you need a CUDA enabled GPU and the following software installed on your system:

* `Docker`
* `Docker Compose`
* `Taskfile`

## HOW TO REPRODUCE THE EXAMPLE

The repository includes two people trackers that we use to show how to run this software against the 23 sequences of the dataset and interpret the produced results. The first tracker (TR) relies solely on RGBD cameras while the second (TL) solely on lidar scans. The trackers use the same identity-tracking component, based on [Norfair](https://github.com/tryolabs/norfair) without re-identification, but different detection components: TR detects people using [YOLOv9](https://docs.ultralytics.com/models/yolov9/) and estimates their 2D projection on the floor using the depth component of the images; TL relies on the [DR-SPAAM](https://arxiv.org/abs/2004.14079) detector instead. In the following we report HOTA, a metric based on IoU to quantify the match between estimations and ground truth tracks, and the three metrics that it summarizes: *LocA* (how well the tracker locates targets), *DetA*  (how well it detects people), and
*AssA* (how well it keeps track of identities over time). We include additional metrics *DetRe* (detection recall), *DetPr* (detection precision), *AssRe* (association recall), and *AssPr* (association precision).

| Tracker | HOTA | LocA | DetA | AssA | DetRe | DetPr | AssRe | AssPr |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| TR | **0.26** | **0.70** | 0.26 | **0.26** | 0.30 | **0.47** | **0.30** | **0.53** |
| TL | 0.20 | 0.67 | 0.22 | 0.19 | 0.35 | 0.29 | 0.24 | 0.42 |

Detailed per-sequence metrics are available in:
- [evaluation-output/lidar.csv](evaluation-output/lidar.csv)
- [evaluation-output/rgbd.csv](evaluation-output/rgbd.csv)

On average,  TR  performs better than  TL. Analysis of individual sequences leads to further insight. For instance,  TR  performs worse when there is a single subject close to the platform (sequence 1):  it misses many detections (low *DetRe*) and, when the person is detected, the accuracy of the detection is below average (low *DetPr*). When one person moves far away from the platform (3),  TR  also performs poorly: depth calculation degrades (low *DetPr*) when the subject is far. The ``office'' sequences are also challenging for TR: low *DetRe* signals that recall of RBG-based recognition algorithm decreases in the presence of environmental objects that add complexity to the scene and partially obstruct people. 

The performance of  TL  is lower but more consistent across sequences.
We observe that the tracker struggles to identify people (low *AssA*) when people are grouped closely, like in sequences 6 and 7.

To run the trackers against these sequences follow this procedure download the sequences.zip file from [Zenodo](https://zenodo.org/records/16418518) and unzip it in the root folder of the repository. Then run the following commands from the root folder of the repository replacing `<sensor>` with rgbd or lidar:

```sh
task setup 
task run-<sensor>-tracker
task run-evaluation
```

The results of the evaluation will be saved in `evaluation-data/trackers`, where
`pedestrian_plot.pdf` reports a summary on all sequences and `pedestrian_detailed.csv` contains
the details for each sequence. Note that results may vary slightly due to the
non-determinism of multi-processing and inter-process communications.

To make a quicker test there is also the possiblity to run TR on a single sequence: download the sequences-test.zip file from the supplementary materials of the paper and unzip it in the root folder of the repository. Then  run the following commands from the root folder of the repository:

```sh
task setup
task run-trackers-test
task run-evaluation-test
```

The output of the evaluation will be saved in `evaluation-data-test/trackers`.

## HOW TO RUN THE BENCHMARK ON A CUSTOM TRACKER

To benchmark a ROS 2 compatible people tracker, follow these steps:

* write a Docker-Compose file that runs your tracker. The tracker must read the RGBD cameras and the lidar data from the dedicated ROS 2 topics that are listed in the following table:

| Topic | Message Type | Description |
|---|---|---|
| `/camera_<i>/color/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB intrinsic parameters of camera i=1, 2 |
| `/camera_<i>/color/image_raw` | `sensor_msgs/msg/Image` | RGB images of camera i=1, 2 |
| `/camera_<i>/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth intrinsic parameters of camera i=1, 2 |
| `/camera_<i>/depth/image_rect_raw` | `sensor_msgs/msg/Image` | Depth data from camera i=1, 2 |
| `/camera_<i>/gyro/sample` | `sensor_msgs/msg/Imu` | IMU of camera i=1, 2 |
| `/camera_<i>/accel/sample` | `sensor_msgs/msg/Imu` | Accelerometer of camera i=1, 2 |
| `/scan` | `sensor_msgs/msg/LaserScan` | Laser scans |

* the tracker must publish the data on a dedicated topic named `/tracks`. The messages published must be of type `rexasi-trackers/ros/rexasi_tracker_msgs/msg/Tracks.msg`. Note that to run the benchmark the mandatory fields are `identities` and `centers`. The former is the list of the track IDs while the latter is the list of their current positions;
* in `rexasi-trackers/.env`, set the variable `TRACKER_COMPOSE_FILE` to the absolute path of the Docker-Compose file that runs your tracker;
* run `task run-custom-tracker` to execute the tracker against all the sequences;
* run `task run-evaluation` to evaluate the results of the tracker.

The results of the evaluation will be saved in `evaluation-data/trackers`, where `pedestrian_plot.pdf` reports a summary on all sequences and `pedestrian_detailed.csv` contains the details for each sequence.

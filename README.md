# msmpt-tools

This repository provides the code associated with the paper titled "Benchmarking Multi-Sensor Multi-People Tracking for Ground Robots: Dataset and Tools" that is currently under revision at IEEE Robotics and Automation Practice (RA-P).

## Requirements

To run the code you need a CUDA enabled GPU and the following software installed on your system:

* `Docker`
* `Docker Compose`
* `Taskfile`

## HOW TO REPRODUCE THE EXAMPLE

The repository includes two people trackers that have been benchmarked in section V-A of the paper. You can use them to reproduce the results we report in Table II or to perform a test on a short sequence.

* Test sequence: to run the RGBD tracker on the sequence contained in sequences-test and measure its performance, you need to download the sequences-test.zip file from the supplementary materials of the paper and unzip it in the root folder of the repository. Then
 run the following commands from the root folder of the repository:

```sh
task setup
task run-trackers-test
task run-evaluation-test
```

The output of the evaluation will be saved in `evaluation-data-test/trackers`.

* All sequences: to reproduce the results of Section V-A, download the sequenes.zip file from [IEEEDataPort](https://ieee-dataport.org/documents/benchmarking-multi-sensor-multi-people-tracking-ground-robots-dataset-and-tools) and unzip it in the root folder of the repository. Then run the following commands from the root folder of the repository replacing `<sensor>` with rgbd or lidar:

```sh
task setup 
task run-<sensor>-tracker
task run-evaluation
```

The results of the evaluation will be saved in `evaluation-data/trackers`, where
`pedestrian_plot.pdf` reports a summary on all sequences and `pedestrian_detailed.csv` contains
the details for each sequence. Note that results may vary slightly due to the
non-determinism of multi-processing and inter-process communications.

# ROS Automated License Plate Recognition

## Setup

### Prerequisites

* ROS (tested with ROS Melodic)
* OpenALPR

### Installation

* Create a workspace directory, `catkin_ws`

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
```

* Prepare the workspace

```bash
$ wstool init src
```

* Copy the `myrosalpr` ROS package to the workspace, at `~/catkin_ws/src/`

* Build the workspace and the package
  
```bash
$ cd ~/catkin_ws
$ catkin_make
```
* Install the package

```bash
$ catkin_make install
```

## Running the package

This package is a subscriber node that listens to the `cv_camera/image_raw` topic from publishers, performs license plate recognition using OpenALPR, and outputs the results in JSON format.

The executable for the package can be found at `~/catkin_ws/install/lib/myrosalpr/myrosalprengine`.

Usage:
```bash
$ ~/catkin_ws/install/lib/myrosalpr/myrosalprengine --result result.json
```

where `result.json` stores the license plate predictions in JSON format.

## License
This code has been provided under the [3-clause BSD License](LICENSE).

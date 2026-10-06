# Kalman Filter

A 2D state estimation and tracking engine built in C++ using [Eigen](https://libeigen.gitlab.io/). It implements a numerically stable linear Kalman filter capable of extracting clean trajectories from highly noisy sensor data and prolonged sensor blackouts. Data visualization is handled via Python and [Matplotlib](https://matplotlib.org/).

## Features

* **Header-only template design** (`kf.hpp`) allowing for easily configurable state and measurement vector dimensions.
* **Numerically stable updates** leveraging the Joseph form for covariance updates and an LDLT solver to prevent floating-point destabilization.
* **Real-world data parsing** to track asynchronous motion using the [TUM RGB-D SLAM dataset](https://cvg.cit.tum.de/data/datasets/rgbd-dataset).
* **Synthetic trajectory generation** supporting time-varying control inputs.
* **Edge-case handling** for dynamic time steps and simulated sensor failure.

## Examples

### Synthetic Control Simulations

<p align="center">
  <img src="https://github.com/Oppenjaimer/kalman-filter/blob/master/images/lissajous.png?raw=true" width="48%">
  <img src="https://github.com/Oppenjaimer/kalman-filter/blob/master/images/loops.png?raw=true" width="48%">
  <br>
  <img src="https://github.com/Oppenjaimer/kalman-filter/blob/master/images/spiral.png?raw=true" width="48%">
  <img src="https://github.com/Oppenjaimer/kalman-filter/blob/master/images/sensor_blackout.png?raw=true" width="48%">
  <br>
  <em>
    Top-left: Lissajous | Top-right: Loops<br>
    Bottom-left: Spiral | Bottom-right: Sensor Blackout
  </em>
</p>

### Real-World Dataset Tracking

<p align="center">
  <img src="https://github.com/Oppenjaimer/kalman-filter/blob/master/images/xyz_dataset.png?raw=true" width="48%">
  <img src="https://github.com/Oppenjaimer/kalman-filter/blob/master/images/floor_dataset.png?raw=true" width="48%">
  <br>
  <img src="https://github.com/Oppenjaimer/kalman-filter/blob/master/images/room_dataset.png?raw=true" width="48%">
  <img src="https://github.com/Oppenjaimer/kalman-filter/blob/master/images/pioneer_dataset.png?raw=true" width="48%">
  <br>
  <em>
    Top-left: <code>fr1/xyz</code> | Top-right: <code>fr1/floor</code><br>
    Bottom-left: <code>fr1/room</code> | Bottom-right: <code>fr2/pioneer_slam</code>
  </em>
</p>

## Building

The project uses a `Makefile` and requires `g++` to compile, as well as [Eigen](https://libeigen.gitlab.io/).

To generate the executable binaries, run `make` in the root directory. This will automatically detect the source files and output the following standalone executables in the `bin/` directory:

* `bin/control_sim`: Generates and tracks synthetic paths using parametrized control inputs.
* `bin/dataset_track`: Parses and tracks the real-world TUM dataset utilizing a constant velocity model.

Object files are placed in the `build/` directory.

## Usage

To track the synthetic control inputs, execute the simulation binary.

```bash
./bin/control_sim
```

To run the real-world trajectory tracker, execute the dataset binary. If run without arguments, it will launch an interactive menu to select any `.txt` dataset located in the `data/` directory.

```bash
./bin/dataset_track [DATASET]
```

The tracking data will be saved to `data/data.csv`. Once the CSV is generated, use the included Python script (requires `pandas` and `matplotlib`) to visualize the 2D trajectory and the state position graphs alongside their corresponding 3σ bounds.

```bash
python scripts/plot.py
```

## License

This project is available under the MIT License.

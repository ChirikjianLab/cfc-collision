# CFC: Collision detection based on closed-form contact space parameterization
[![C++](https://github.com/ChirikjianLab/cfc-collision/actions/workflows/github-action-ci-basic.yml/badge.svg)](https://github.com/ChirikjianLab/cfc-collision/actions/workflows/github-action-ci-basic.yml)
[![Clang-Tidy](https://github.com/ChirikjianLab/cfc-collision/actions/workflows/github-action-ci-clang-tidy.yml/badge.svg)](https://github.com/ChirikjianLab/cfc-collision/actions/workflows/github-action-ci-clang-tidy.yml)

Collision detection, distance queries (penetration depth), closes points computations via closed-form contact space (CFC) for unions of convex bodies with smooth boundaries.

## Introduction
This is the C++ implementation for the narrow phase collision detection problem between two general unions of convex bodies encapsulated by smooth surfaces. The approach, namely CFC (Closed-Form Contact space), is based on parameterizing their contact space in closed-form. The first body is dilated to form the contact space while the second is shrunk to a point. Then, the collision detection is formulated as finding the closest point on the parametric contact space with the center of the second body. Numerical solutions are proposed based on the point-to-surface distance as well as the common-normal concept. Furthermore, when the two bodies are moving or under linear deformations, their first time of contact is solved continuously along the time-parameterized trajectories. Benchmark studies are conducted for the proposed algorithms in terms of solution stability and computational cost.

- Paper: [IEEE Robotics and Automation Letters (RA-L)](https://ieeexplore.ieee.org/document/9829274)
- Project page: [https://chirikjianlab.github.io/cfc-collision-page/](https://chirikjianlab.github.io/cfc-collision-page/)
- Application code repository: [https://github.com/ruansp/cfc_collision_app](https://github.com/ruansp/cfc_collision_app)
- MATLAB implementation: [https://github.com/ChirikjianLab/cfc-collision-matlab](https://github.com/ChirikjianLab/cfc-collision-matlab)
- Data: [Benchmark data in the paper](https://drive.google.com/drive/folders/17jSSC-EIhiSTqXSgfoEOs4R7mzKy1d1i?usp=sharing)

## Authors
[Sipu Ruan](https://ruansp.github.io), Xiaoli Wang and [Gregory S. Chirikjian](https://scholar.google.com/citations?user=qoIuyMoAAAAJ&hl=en)

## Dependency
### Required for core library
- [Ceres solver](http://ceres-solver.org/installation.html) (>= 2.0): Solver for nonlinear least-squares optimization
- Eigen3 
```sh
[sudo] apt install libeigen3-dev
```
- Boost (>= 1.71) 
```sh
[sudo] apt install libboost1.71.0-all-dev
```

### Optional for testing and benchmark
- [ifopt](https://github.com/ethz-adrl/ifopt): Solver for constrained optimization using interior-point method
- [FCL](https://github.com/flexible-collision-library/fcl) (>= 0.6): Flexible collision library
- [libccd](https://github.com/danfis/libccd.git): Dependency for FCL
- google-test: Unit test tool
```sh
[sudo] apt install libgtest-dev
```

**Note** 
The script to automatically install the above dependencies is provided in "/script/install-dependencies.sh".

## Compilation and installation
The core implementation is a templated header-only library. To use the classes, there is no need to compile but you could simply copy the "/include" directory to the specific path and link/include in your own project correctly.

### Download the repository
- Clone the repository
```sh
git clone https://github.com/ChirikjianLab/cfc-collision.git
```

### Build 
#### Build in localhost
- Installation can follow the standard CMake project. After clonging and go to the source directory,
```sh
mkdir build && cd build
cmake .. && make
```

**Note**
By default, the test and benchmark scripts are built if the correct external libraries are installed and found. To disable testing, add argument in the cmake step
```sh
cmake -DBUILD_TESTING=off ../
```

- To view the unit tests after compilation,
```sh
make test
```

#### Build using Docker
- After cloning the repository, go to "/script" folder and start Docker container
```sh
cd /script
chmod +x start_docker.sh
./start_docker.sh
```
It will download the image if for the first time. 

- Then, build the package _inside the container_,
```sh
cd /home/cfc
mkdir build && cd build
cmake .. && make
```

### Installation
After compilation, inside the "/build" directory,
```sh
[sudo] make install
```

To uninstall,
```sh
[sudo] make uninstall
```

**Note**
By default, the header files of the core library will be installed within "/usr/local". You could also specify the folder in the "cmake" step by 
```sh
cmake -DCMAKE_INSTALL_PREFIX=/your/specified/path/ ../
```

## Running instructions
### Command line arguments for benchmarks
Executables for benchmark in both static and continuous cases are located in "/build/test/benchmark" folder.

- [BenchmarkDistanceStatic.cpp](/test/benchmark/BenchmarkDistanceStatic.cpp): Benchmarks for static collision detection.
```sh
./BenchmarkDistanceStatic 100 all 10
```
- [BenchmarkDistanceContinuous.cpp](/test/benchmark/BenchmarkDistanceContinuous.cpp): Benchmarks for continuous collision detection.
```sh
./BenchmarkDistanceContinuous 100 all 10
```

### Arguments for both scripts (in order)
- Number of trials 
- Geometric pairs: e.g. "all" (will run for all geometric pairs), "SQ-SQ" (will run for a specific pair), etc.
- (Optional) Number of sampled vertices on each surface (default = 20).

### Result files
Results are stored in "/data" folder, which is automatically generated at build time. After running the benchmark script, several ".csv" files will be generated to record the results (each row corresponds to one experimental trial):

For static case:
- "bench_config_${GeomType}_s1.csv", "bench_config_${GeomType2}_s2.csv": Configuration for the two bodies.
- "bench_result_${GeomType}_${GeomType}_${StaticMethod}.csv": Benchmark results for different methods.

For continuous case:
- "bench_config_continuous_${GeomType}_s1.csv", "bench_config_continuous_${GeomType2}_s2.csv": Configuration for the two bodies.
- "bench_result_continuous_${GeomType}_${GeomType}_${CCDMethod}.csv": Benchmark results for different methods.

To visualize the benchmark results (i.e., comparisons on running time, accuracy and number of iterations), please refer to the [instructions](https://github.com/ChirikjianLab/cfc-collision-matlab/blob/main/data/README.md) in the repository of MATLAB implementation.

**Note**
- ${GeomType} = {"SQ", "E", "PE"}
- ${StaticMethod} = {"CFCFixedPoint", "CFCLeastSquares", "CFCLeastSquaresCommonNormal", "FCL", "Implicit", "CommonNormalFixedPoint", "CommonNormalLeaseSquares"}
- ${CCDMethod} = {"CFCLeastSquaresTran", "CFCLeastSquaresLinear"}

## Reference
If you find our work useful in your research, please consider citing:

- S. Ruan, X. Wang and G. S. Chirikjian, "Collision Detection for Unions of Convex Bodies With Smooth Boundaries Using Closed-Form Contact Space Parameterization," in IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 9485-9492, Oct. 2022, doi: 10.1109/LRA.2022.3190629.

- BibTeX
```
@ARTICLE{ruan2022collision,
  author={Ruan, Sipu and Wang, Xiaoli and Chirikjian, Gregory S.},
  journal={IEEE Robotics and Automation Letters}, 
  title={Collision Detection for Unions of Convex Bodies With Smooth Boundaries Using Closed-Form Contact Space Parameterization}, 
  year={2022},
  volume={7},
  number={4},
  pages={9485-9492},
  doi={10.1109/LRA.2022.3190629}}
```


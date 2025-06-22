# Outset
C++ space and orbit sims

## Dependencies

<div align="center">

| Core Dependencies | Version | Description |
|------------------|---------|-------------|
| ![C++](https://img.shields.io/badge/C++-17+-00599C?style=flat&logo=c%2B%2B) | 17+ | Core language |
| ![CMake](https://img.shields.io/badge/CMake-3.10+-064F8C?style=flat&logo=cmake) | 3.10+ | Build system |
| ![Eigen](https://img.shields.io/badge/Eigen-3-0056B3?style=flat) | 3.x | Linear algebra (included as submodule) |
| ![GTest](https://img.shields.io/badge/GoogleTest-latest-success?style=flat) | Latest | Testing framework |
<!-- | ![ERFA](https://img.shields.io/badge/ERFA-latest-orange?style=flat) | Latest | Astronomy routines | -->

</div>

### Platform-Specific Setup
<details>
<summary>macOS</summary>

```bash
brew install cmake googletest
```
</details>

<details>
<summary>Linux (Ubuntu/Debian)</summary>

```bash
sudo apt-get install cmake libgtest-dev
```
</details>


## Building and Running

1. Clone the repository and initialize submodules:

```bash
git clone https://github.com/Cesium-Lab/Outset.git
cd Outset
```

2. Create and enter build directory:

```bash
mkdir build && cd build
```

3. Configure and build the project:

#### macOS/Linux
```bash
cmake ..
make
```
<!-- #### Windows (using Visual Studio)
```bash
cmake -G "Visual Studio 17 2022" ..
cmake --build . --config Release
``` -->

4. Run the simulation:

#### macOS/Linux
```bash
./Outset
```
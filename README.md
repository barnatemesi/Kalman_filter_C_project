# Kalman-filter project
This project is based on the following Matlab / project: Linear State Estimators for Load Estimation in a PMSM system.

[Linear state estimators](https://github.com/barnatemesi/PMSM_FOC_LTID_Linear)

This project achieves the following goals:

- Implement the already designed Kalman-filter in C. The implementation shall be as re-useable as possible.

- Study the ways of optimizing matrix calculations in microcontrollers.

The project is set up to accomodate systems of up to 5th order. Technically, the project is suitable for systems of up to ~12th order.
If the order of your modelled system is larger than 10, unfortunately, this is implementation is not for you.

The project contains doxygen style commenting!

Versioning is based on: 
[semantic versioning](https://semver.org/)

# Install
- GCC (latest) for testing

- Your desired compiler. Based on your project, hardware etc.

- make

- optionally: cmake (TBD)

# Getting started
Download the repo.
 
Copy the folder `kalman_filter` to your project. 

Copy the folder `data` to your project. 

Set your system parameters in `kalman_filter.h`, where `NUMOFROWS` defines the order of the system.

Replace the contents of `matrix_data.c` with the matrices calculated based on your project. Optionally, delete `SYSTEM_SWITCH`.

Copy the contents of the folder `helper_files`, to your own project where you collect such files. Optionally, copy the header files to your `/inc` folder and merge `helper_files` with your version of such source / header file(s).

If your project is based on an ARM microcontroller and CMSIS, define the following symbol: `-DCPU_IS_ARM`.

Note: further optimization might be possible based on your hardware, specially if it's not based on an ARM microcontroller. Please do your due diligence in this regard!

Optionally, re-use functions from `helper_files.h` and `matrix_compute.h` in your project.

# Kalman-filter project description
This project is based on the following Matlab / project: Linear State Estimators for Load Estimation in a PMSM system.

[Linear state estimators](https://github.com/barnatemesi/PMSM_FOC_LTID_Linear)

This project achieves the following goals:

- Implement the already designed Kalman-filter in C. The implementation shall be as re-useable as possible.

- Study the ways of optimizing matrix calculations in microcontrollers.

The project is set up to accomodate systems of up to 5th order. Technically, the project is suitable for systems of up to ~12th order.
If the order of your modelled system is larger than 10, unfortunately, this is implementation is not for you.

The project contains doxygen style commenting!

Static analyzer of the project:

Cppcheck - ver 2.16

[cppcheck](https://cppcheck.sourceforge.io/)

Versioning is based on: 

[semantic versioning](https://semver.org/)

# Dependencies
- CMake - minimum v3.20.0

- MinGW (tested with 13.2.0) or your desired compiler

- optionally: Doxygen (minimum v1.9.7)

# Getting started
Add to your Zephyr project such as:
```yaml
manifest:
  projects:
    - name: Kalman_filter_load_estimation_project
    description: Kalman-filter implementation to estimate load torque state
    revision: v1.2.0
    path: modules/lib/kalman-filter-load-estimation # <your/desired/path>
    url: https://github.com/barnatemesi/Kalman_filter_C_project.git
```


# Usage
`#include <kalman_filter.h>`

The definition of the API is found here: `kalman_filter.h`. 

If your project is based on an ARM microcontroller and CMSIS, define the following symbol: `-DCPU_IS_ARM`.

Note: further optimization might be possible based on your hardware, specially if it's not based on an ARM microcontroller. Please do your due diligence in this regard!

Optionally, re-use functions from `helper_files.h` in your project.

# Sources

The main motivation of this project is to study efficient Kalman-filter algorithms. Inspired by:

https://www.youtube.com/watch?v=WgTr79htuM4&t=633s&ab_channel=DepthBuffer

Code and technical methodology are inspired by:

https://ch.mathworks.com/help/control/ref/ss.html

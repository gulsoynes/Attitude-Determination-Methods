

[![Stargazers][stars-shield]][stars-url]
[![LinkedIn][linkedin-shield]][linkedin-url]


<!-- ABOUT THE PROJECT -->
# About The Project
This project presents the modeling, simulation and comparison of attitude determination algorithms for a nano-satellite in MATLAB.


## Single Frame Methods and Kalman Filter Based Nanosatellite Attitude Estimation

Project contains 4 different attitude determination algorithms : TRIAD and Davenport's q method, and their integrated filtering counterparts, TRIAD+EKF and q-method+EKF.

The attitude dynamics of a Low Earth Orbit (LEO) satellite were examined using chosen initial conditions. Sensor measurements were modeled for a sun sensor, magnetometer, and nadir sensor. The algorithms were tested using dual combinations of these sensor data, with the q method also tested using all available sensor informations.

### Built With

* [![MATLAB][MATLAB]][MATLAB-url]


### Prerequisites

Make sure that MATLAB and Simulink R2023a or newer version is already downloaded. Some functions or blocks will not work in older versions.



<!-- USAGE EXAMPLES -->
## Usage

`Main.m` : Main Simulation

`TRIAD.m` : TRIAD Algorithm

`qMethod.m` : Davenport`s q Method Algorithm

`EKF_plus.m` : Extended Kalman Filter Algorithm

### Functions 

`SunSensor.m` : Sun Sensor Model

`Magnetometer.m` : Magnetometer Sensor Model

`HorizonSensor.m` : Horizon Sensor Model

`F_Matrix.m` : Linearizaton of Dynamics

`R_Matrix.m` : Linearized Noice Covariance Matrix for Hibrid Kalman Filtering

`CtoEuler.m` : Convert Rotation Matrix to Euler Angles (roll, pitch, yaw)

`Ctoq.m` : Convert Rotation Matrix to Quaternion

`qtoC.m` : Convert Quaternion to Rotation Matrix

`qtoEuler.m` : Convert Quaternion to Euler Angles

`JDate.m` : Julian Date calculator

`PLOTTING.m` : Plotting

Note : `CtoEuler.m`, `Ctoq.m`, `qtoC.m`, `qtoEuler.m`, `JDate.m` functions can be replaceable with built-in MATLAB functions. They were written because no built-in functions were tried to be used in the project. 

Please refer to the [Report](https://github.com/gulsoynes/Attitude-Determination-Methods/blob/main/Graduation_Project_Final.pdf). Essential formulation and details and results of simulation are reported. Since codes were uploaded in the early stages of the project they may contain errors. For more information, please do not hesitate to contact me.



<!-- CONTACT -->
## Contact

Neslihan Gülsoy - gulsoyneslihan0@gmail.com



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->

[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/neslihan-gülsoy

[MATLAB]: https://img.shields.io/badge/matlab-001000?style=for-the-badge&logo=matlab&logoColor=blue
[MATLAB-url]: https://www.mathworks.com/products/matlab.html

[SIMULINK]: https://img.shields.io/badge/simulink-001000?style=for-the-badge&logo=matlab&logoColor=blue
[SIMULINK-url]: https://www.mathworks.com/products/simulink.html

[stars-shield]: https://img.shields.io/github/stars/gulsoynes/Attitude-Determination-Methods.svg?style=for-the-badge
[stars-url]: https://github.com/gulsoynes/Attitude-Determination-Methods/stargazers

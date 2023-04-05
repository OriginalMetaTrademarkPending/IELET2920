# IELET2920
Bachelor thesis on the research and development of next-generation handgrip trainers.

# Description of the task
In essence, the task is composed of the following parts:
- Mathematical modelling of the system. Due to the nature of this thesis, the model cannot be parameterized through a theoretical approach.
- Estimation of the parameters of the model. Referring to the point above, a least squares estimator has been used to estimate the model parameters.
- Kalman filter design. This is the final point of the design, which aims at improving the measurements, as well as estimating the hidden variable (fatigued muscle mass).

Here it is important to note that the task includes a significant amount of hardware design, especially in the domain of 3D-modelling, 3D-printing and PCB design.

# Parameter estimation

Until now, 5 tests have been completed. The results are shown below:

## Test 1: data.csv

$\theta_{af} = 0.0258$
$\theta_{ar} = 12.9653$
$\theta_{ra} = 1.8997$
$\theta_{fa} = 0.0230$
$M = 7.5095$
Sum of squares: 238.7625

## Test 2: test1.csv

$\theta_{af} = 0.0427$
$\theta_{ar} = 341.1526$
$\theta_{ra} = 3.1696$
$\theta_{fa} = 0.0747$
$M = 6.3241$
Sum of squares: 305.6513

## Test 3: test2.csv

$\theta_{af} = 0.0216$
$\theta_{ar} = 29.7083$
$\theta_{ra} = 4.9487$
$\theta_{fa} = 0$
$M = 7.4149$
Sum of squares: 297.8530

## Test 4: test3.csv

$\theta_{af} = 1.2019$
$\theta_{ar} = 75.4510$
$\theta_{ra} = 2.3210$
$\theta_{fa} = 1.0925$
$M = 12.1936$
Sum of squares: 954.0013

## Test 5: test4.csv

$\theta_{af} = 0.1093$
$\theta_{ar} = 517.0190$
$\theta_{ra} = 5.3904$
$\theta_{fa} = 0.1584$
$M = 5.9675$
Sum of squares: 767.8368
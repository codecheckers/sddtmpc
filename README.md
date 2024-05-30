# Implementation of SDD-TMPC (state-dependent dynamic tube model predictive control) for a nonlinear problem

## Project description 

The code in this repository is used to implement and test a new algorithm called SDD-TMPC for a nonlinear system. It is a control algorithm that sacrifices some optimality (much less than tube MPC), but provides robust solutions (more about this can be found in [2]). In this repository, NTMPC (from [1]) and SDD-TMPC have been implemented to control a unicycle robot. To run the code, you have to execute the main file. All parameters (robot's maximum speed, MPC's horizon, etc.) can be found in the main file. The "figures" folder contains all the figures. The final data is stored in "final_data.mat".


## How to use

To use the code, you need Matlab (the code has been tested for 2022a) and the listed dependencies. To run the simulation, you need to run the main.m. Using parameters, it is possible to investigate the behavior of the controller for other problems. All necessary information can be found in the comments. 

## Dependencies

Matlab's optimization toolbox (to use interior-point algorithm) - https://nl.mathworks.com/products/optimization.html

Matlab's global optimization toolbox (to use particle swarm) - https://nl.mathworks.com/products/global-optimization.html


## Known problems

The software needs a lot of time to run the simulation because SDDTMPC needs more than 20 seconds to solve the optimization problem which was solved with particle swarm algorithm. It is a global optimization algorithm and the problem is nonlinear, so we cannot guarantee that the global minimum will be found. Because of this, the final trajectories depend on the random generator. We use the solution of NTMPC as a starting set of decision variables, but it is not a perfect solution and multi-start would be too time-consuming.


## License

The software is distributed under the MIT license:


Copyright (c) 2023 Filip Surma

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE

## References

1. Sun Z, Dai L, Liu K, Xia Y, Johansson KH. Robust MPC for tracking constrained unicycle robots with additive disturbances.
Automatica 2018;90:172–184. https://www.sciencedirect.com/science/article/pii/S0005109817306350

2. Surma, F. Jamshidnejad, A.(2023). State-Dependent Dynamic Tube MPC: A Novel tube MPC Method with a Fuzzy Model of Disturbances [Manuscript submitted for publication]. Aerospace Engineering, Tu Delft.


# Multi-Agent System Simulation Library

## Basic Structure

The base of this simulation tool are the [`BaseAgent`](lib/agents/BaseAgent.m) and [`BaseNetwork`](lib/networks/BaseNetwork.m) classes.
These abstract classes specify the interface that is used to drive the simulation but cannot be used for simulation themselves.
To perform a simulation, you must define a set of classes that inherit from [`BaseAgent`](lib/agents/BaseAgent.m) and [`BaseNetwork`](lib/networks/BaseNetwork.m).

Currently the project contains four such implementations for [`BaseNetwork`](lib/networks/BaseNetwork.m), [`IdealNetwork`](lib/networks/IdealNetwork.m), [`BernoulliNetwork`](lib/networks/BernoulliNetwork.m), [`MarkovNetwork`](lib/networks/MarkovNetwork.m) and [`SinrNetwork`](lib/networks/sinr/SinrNetwork.m).
The first implements an ideal communication network without packet loss but an optionally finite transmission range, the second additionally models packet loss using a Bernoulli distribution and the third uses a 2-state Markov model for each transmission channel.
The [`SinrNetwork`](lib/networks/sinr/SinrNetwork.m) simulates the communication channels with a signal-to-interference-plus-noise-ratio based model.
For further information on the fourth, see [Using the SINR Networking Library](#using-the-sinr-networking-library) below.

For a high simulation speed, it is important to select the transmission range appropriatly, as interactions between agents should be minimized.
Selecting the range to large can drastically decrease the simulation performance by introducing quadratic scaling in the number of agents.

To model the dynamic behaviour of the agents, an extension of the [`BaseAgent`](lib/agents/BaseAgent.m) class is required.
A variety of dynamic models to choose from is defined in [`lib/agents/models`](lib/agents/models), which can be used to simplify the implementation of the simulation.
If the desired model is contained in the collection, only the specific controller behaviour needs to be implemented manually.
Otherwise, the folder [`lib/dynamics`](lib/dynamics) contains building blocks for custom implementations of LTI, LPV and general nonlinear dynamics, each in discrete- and continuous-time.
Whenever possible, the agent dynamics should be implemented in discrete-time because the discrete-time evaluation is orders of magnitudes faster.

## Usage

There a three main steps to implementing a simulation within this project.

1. Implement a suitable networking class by extending [`BaseNetwork`](lib/networks/BaseNetwork.m).
   For most cases, the network implementations provided by the project should suffice, so check these first before implementing your own.
2. Implement the desired agent behaviour by extending [`BaseAgent`](lib/agents/BaseAgent.m).
   If the desired model is defined in [`lib/agents/models`](lib/agents/models), you only need to extend the `step()` function that implements the controller behaviour.
   For examples, see [`ObstacleAvoidingAgent`](examples/flocking_with_obstacles/ObstacleAvoidingAgent.m), [`FormationQuadrotor`](examples/lti_formation_control/FormationQuadrotor.m) or [`FormationUnicycle`](examples/lpv_formation_control/FormationUnicycle.m).
   Otherwise, specify the dynamics manually, where you, in addition to the `step()` function, need to provide implementations for `position` and `velocity`.
   This manuell implementation can use the dynamic systems defined in [`lib/dynamics/`](lib/dynamics).
3. Implement the main simulation loop by calling `Agent.step()` and `Network.process()` in lockstep.
   The [`SimulationManager`](lib/SimulationManager.m) can help you accomplish that, especially with multi rate simulations.
   For convenience, the [`DataLeech`](/lib/util/DataLeech.m) class is provided to help with saving class properties, e.g. positions and velocities of the agents.

## Example Simulations

In the [`examples`](examples) folder, you find several example simulations build with this library.
There are currently the following examples

* [First-order Consensus](examples/first_order_consensus):
Implements a first order consensus protocol with ideal communication.
* [Obstacle Avoidance](examples/flocking_with_obstacles):
Simulation of flocking behaviour with obstacles that need to be avoided.
Packet loss is simulation using the SINR model.
* [Quadrocopter Formation](lti_formation_control):
Formation control for linearized quadrotor models in 3D space.
Packet loss is simulation using a Bernoulli loss model.
* [Unicycle Formation](lpv_formation_control):
Formation control for dynamic unicycle models that follow a reference trajectory using a leader-follower structure.
Packet loss is simulation using the SINR model.

Executing the `Simulation.m` script will run the simulation and animate the result.

## Using the SINR Networking Library

The [`SinrNetwork`](lib/networks/sinr/SinrNetwork.m) internally uses a C++ network simulation library for adequate computational performance.
For Matlab to be able to call the networking code, the library is wrapped in a compiled MEX interface.
All this is transparant to users of the library, but the computer running the simulation will have to be prepared if you want to run the SINR based network in addition to the idealized models.

Due to the way the MEX interface interacts with C++ classes, its only possible to create a single instance of the [`SinrNetwork`](lib/networks/sinr/SinrNetwork.m) class at at time.
Should you need to create a new instance from scratch, the old object can be cleared using the [`delete`](https://mathworks.com/help/matlab/ref/delete.html) function.
However, it is still possible to use the [`SinrNetwork`](lib/networks/sinr/SinrNetwork.m) with the *ParallelComputingToolbox* because that spawns multiple headless Matlab instances in the background such that each Matlab instance only has a single copy of the class.

### Setting up the Matlab MEX Compiler

Before running simulations with the SINR library, the C++ library and its Matlab MEX wrapper have to be compiled first.
This is done automatically when constructing a [`SinrNetwork`](lib/networks/sinr/SinrNetwork.m) object if Matlab is set up correctly.
To configure Matlab, run the following command:

```matlab
mex -setup C++
```

Matlab should print something like

> MEX configured to use 'MinGW64 Compiler (C++)' for C++ language compilation.

if it is configured correctly.
Otherwise you probably need to install a C++ compiler for your operating system.
If you are working on Windows, you can install a suitable compiler from the Matlab *Add-On Explorer*.
Search for [MATLAB Support for MinGW-w64 C/C++ Compiler](https://de.mathworks.com/matlabcentral/fileexchange/52848-matlab-support-for-mingw-w64-c-c-compiler) from within Matlab, install the package and afterwards run the `mex` command again.

When you construct a SinrNetwork object, Matlab should now print the following in the command window.

> SINR networking library needs to be rebuild.
>
> Building...
>
> Building succeeded!

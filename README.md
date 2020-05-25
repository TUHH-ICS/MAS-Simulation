# MAS Simulation Library

## Basic Structure

The base of this simulation tool are the `BaseAgent` and `BaseNetwork` classes, which you find in the `lib` folder.
These abstract classes specify the interface that is used to drive the simulation but cannot be used for simulation themselves.
To perform a simulation, you must define a set of classes that inherit from `BaseAgent` and `BaseNetwork`.

Currently the project containts only one such implementation, `IdealNetwork`.
It is a very simple network implementation that does not consider any non-ideal network properties and broadcasts the messages over a finite range.
For a high simulation speed, it is important to select the communication range appropriatly, as interactions between agents should be minimized.
Selecting the range to large can drastically decrease the simulation performance by introducing quadratic scaling in the number of agents.

At the moment, there is no such conveniency class implementing `BaseAgent` but at least an `LtiAgent` and probably also an `LpvAgent` should be added after some design work.
Further work is also required to consider *time-triggered* and *event-triggered* control.

## Usage

There a three main steps to implementing a new simulation within this project.

1. Implement a suitable networking class by extending `BaseNetwork`.
   For most cases, the network implementations provided by the project should suffice, so check these first before implementing you own.
2. Implement the desired agent behaviour by extending `BaseAgent`.
   The main work is in implementing the `step()` function that calculates a single discrete timestep for the agent and some minor work is in defining the projections `position` and `velocity`.
   For an example, see `FlockingAgent` in `examples/flocking`.
3. Implement the main simulation loop by calling `Agent.simulate()` and `Network.process()` in lockstep.
   At each timestep, you should save the current position of the agents if you want to animate their movement later on.

## Example Simulation

In the `examples` folder, you find an example flocking simulation.
Executing the `Simulation.m` script will run the simulation and animate the result.
This example shows how a custom agent class can be implemented.

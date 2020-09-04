% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

addpath(genpath('../../lib'))

% Clear workspace to increase repeatability
clear

% Seed the pseudo random number generator. This is required if we want
% reproducible simulation, e. g. for profiling the code.
rng(0);

%% Run tracking simulation
Tf = 200;

hippo = HippoCampusAgent(1, zeros(3,1));
dT = hippo.dT;

steps = ceil(Tf / dT);

% Preallocate storage for simulation results
leech = DataLeech(hippo, steps, 'position', 'state', 'att_d', 'ref', 'f');

% Initialize remaining values for the simulation
t = 0;

% Save start time of the simulation. We want to periodically print the
% progress of the simulation.
lastprint = posixtime(datetime('now'));

tic
% profile on
while t < Tf
    hippo.step();
    t = t + dT;
    
    % Save current position of all agents
    leech.save(t);

    % Print progress every 2 seconds
    if posixtime(datetime('now')) - lastprint >= 1
        lastprint = posixtime(datetime('now'));
        fprintf('Simulation %3.5g%% finished\n', 100*t/Tf)
    end
end
% profile viewer
fprintf("Simulation completed in %.3g seconds!\n", toc);
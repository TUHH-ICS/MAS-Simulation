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
Tf = 500;

hippo = HippoCampusAgent(1, [0; 0; 0]);
dT = hippo.dT;

steps = floor(Tf / dT) + 1;

% Preallocate storage for simulation results
leech = DataLeech(hippo, steps, 'position', 'velocity', 'state', 'ref', 'u');

% Initialize remaining values for the simulation
t = 0;

% Save start time of the simulation. We want to periodically print the
% progress of the simulation.
lastprint = posixtime(datetime('now'));

tic
% profile on
while t <= Tf
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

%% Plot results
figure()
plot(leech.t, leech.data.position)
xlabel('Time t')
ylabel('Positions')
legend('x(t)', 'y(t)', 'z(t)')

figure()
pos = leech.data.position;
ref = leech.data.ref;
plot3(pos(:,1), pos(:,2), pos(:,3))
hold on
plot3(ref(:,1), ref(:,2), ref(:,3))
hold off
xlabel('x(t) Coordinate')
ylabel('y(t) Coordinate')
zlabel('z(t) Coordinate')
legend('HippoCampus', 'Reference')

figure()
subplot(211)
plot(leech.t, leech.data.velocity)
xlabel('Time t')
ylabel('Inertial Frame Velocities')
legend('v_x(t)', 'v_y(t)', 'v_z(t)')
subplot(212)
plot(leech.t, leech.data.state(:, 7:9))
xlabel('Time t')
ylabel('Body Frame Velocities')
legend('v_x(t)', 'v_y(t)', 'v_z(t)')

figure()
plot(leech.t, leech.data.u)
xlabel('Time t')
ylabel('Control Inputs')
legend('f', 'phi', 'theta', 'psi')

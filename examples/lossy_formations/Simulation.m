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

%% Network parameters
agentCount = 30;    % Number of agents in the network
dimension  = 3;    % Dimension of the space the agents move in
dT         = 0.01; % Size of the simulation time steps [s]
Tf         = 5;    % Simulation time [s]

% Type of communication, 1->ideal, 2->Bernoulli, 3->Markov
netType    = 2;

%% Initialize the network
switch netType
    case 1
        range   = Inf;   % Range of the radio communication
        Network = IdealNetwork(agentCount, dT, dimension, range);
    case 2
        range     = 3;    % Range of the radio communication
        pTransmit = 0.1;  % Probability of successful transmission
        symmetric = true; % Symmetric or asymmetric link failure
        Network   = BernoulliNetwork(agentCount, dT, dimension, range, pTransmit, symmetric);    
    case 3
        range     = 3;    % Range of the radio communication
        pFail     = 0.5;  % Probability of a good link failing
        pRecover  = 0.1;  % Probability of a broken link recovering
        symmetric = true; % Symmetric or asymmetric link failure
        Network   = MarkovNetwork(agentCount, dT, dimension, range, pFail, pRecover, [], symmetric);
end

%% Initialize the agents
% To avoid Matlab initializing the array of agents without including the
% required constructor arguments, we first construct a cell array of agents
% and later convert this to a standard Matlab array.
Agents = cell(agentCount, 1);
for i = 1:length(Agents)
    % Randomly place the agents in the square [0,30]^3
    pos = 15 + 30 * (rand(dimension, 1) - 0.5);
    
    % Initialize an agent with the generated initial conditions
    Agents{i} = FormationQuadrotor(Network.getId(), pos, [i; 0; 0]);
end
Agents = [Agents{:}];

%% Perform simulation
% The main work of the simulation is performed by the SimulationManager. It
% calls the agent and network routines in the desired frequency
sim   = SimulationManager(Network, Agents);

% Estimate the number of steps based on the final simulation time
steps = sim.estimateSteps(Tf);

% Preallocate storage for simulation results
leech = DataLeech(Agents, steps, 'position', 'ref');

% Initialize remaining values for the simulation
t = 0;

% Save start time of the simulation. We want to periodically print the
% progress of the simulation.
lastprint = posixtime(datetime('now'));

tic
% profile on
while t < Tf
    t = sim.step();
    
    % Save current position of all agents
    leech.save(t)

    % Print progress every 2 seconds
    if posixtime(datetime('now')) - lastprint >= 1
        lastprint = posixtime(datetime('now'));
        fprintf('Simulation %3.5g%% finished\n', 100*t/Tf)
    end
end
% profile viewer
fprintf("Simulation completed in %.3g seconds!\n", toc);

%% Calculate mean square formation error
mse = zeros(leech.steps,1);
for k = 1:leech.steps
    % The reference is only specifying position relative to the centre of
    % mass of the formation. For that reason, we should also calculate the
    % MSE in that way, relative to the mean position and reference.
    pos = squeeze(leech.data.position(k,:,:));
    ref = squeeze(leech.data.ref(k,:,:));
    pos = pos - mean(pos,2);
    ref = ref - mean(ref,2);
    
    mse(k) = sum((pos(:) - ref(:)).^2);
end

fprintf("Root mean square formation error: %g\n", sqrt(sum(mse)));

%% Animate simulation results
% Compute boundary
x_pos  = squeeze(leech.data.position(:,1,:));
y_pos  = squeeze(leech.data.position(:,2,:));
z_pos  = squeeze(leech.data.position(:,3,:));
bounds = @(x) [min(min(x)), max(max(x))];

t_bounds = bounds(leech.t);

figure()
subplot(311)
plot(leech.t, x_pos)
xlim(t_bounds);
xlabel('Time t')
ylabel('x(t)')

subplot(312)
plot(leech.t, y_pos)
xlim(t_bounds);
xlabel('Time t')
ylabel('y(t)')

subplot(313)
plot(leech.t, z_pos)
xlim(t_bounds);
xlabel('Time t')
ylabel('z(t)')

figure()
plot(leech.t, mse)
xlabel('Time t')
ylabel('MSE(t)')
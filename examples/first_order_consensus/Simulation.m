% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

addpath(genpath('../../lib'))

% Clear workspace to increase repeatability
clear

%% Network parameters
agentCount = 20;   % Number of agents in the network
neighbours = 3;    % Number of neighbours for each agent
dimension  = 2;    % Dimension of the space the agents move in
dT         = 1e-2; % Size of the simulation time steps [s]
Tf         = 3;    % Simulation time [s]

%% Initialize the network
range   = Inf;   % Range of the radio communication
Network = IdealNetwork(agentCount, dT, dimension, range);

%% Initialize the agents
% To avoid Matlab initializing the array of agents without including the
% required constructor arguments, we first construct a cell array of agents
% and later convert this to a standard Matlab array.
Agents = cell(agentCount, 1);
for i = 1:length(Agents)
    % Randomly place the agents in the square [0,30]^2
    pos = 15 + 30 * (rand(dimension, 1) - 0.5);
    
    % Select neighbourhood. For this example, use a circular graph with the
    % next agents being neighbours
    id   = Network.getId();
    hood = id + (1:neighbours);
    
    % Prevent index out of bounds for agent indices
    hood = mod(hood - 1, agentCount) + 1;
    
    % Initialize an agent with the generated initial conditions
    Agents{i} = ConsensusAgent(id, dT, pos, hood);
end
Agents = [Agents{:}];

%% Perform simulation
% The main work of the simulation is performed by the SimulationManager. It
% calls the agent and network routines in the desired frequency
sim   = SimulationManager(Network, Agents);

% Estimate the number of steps based on the final simulation time
steps = sim.estimateSteps(Tf);

% Preallocate storage for simulation results
leech = DataLeech(Agents, steps, 'position');

% Simulate!
tic
t = 0;
while t < Tf
    t = sim.step();
    
    % Save current position of all agents
    leech.save(t)
end
fprintf("Simulation completed in %.3g seconds!\n", toc);

% Due to the multi-rate support, the sampling will not always be uniform.
% Therefore, we need to resample the data. The function uses a ZOH
% resampling approach. The resulting figure will be drawn with 20 FPS 
[t_sampled, sampled] = leech.resample(1/20);

%% Animate simulation results
figure()

% Compute boundary
x_pos  = squeeze(leech.data.position(:,1,:));
y_pos  = squeeze(leech.data.position(:,2,:));
bounds = @(x) [min(min(x)), max(max(x))];

for k = 1:length(t_sampled)
    pos   = squeeze(sampled.position(k,:,:));
    scatter(pos(1,:), pos(2,:))
    xlabel('x(t)')
    ylabel('y(t)')
    
    xlim(bounds(x_pos))
    ylim(bounds(y_pos))
    drawnow limitrate
end

t_bounds = bounds(leech.t);

figure()
subplot(211)
plot(leech.t, x_pos)
xlim(t_bounds);
xlabel('Time t')
ylabel('x(t)')

subplot(212)
plot(leech.t, y_pos)
xlim(t_bounds);
xlabel('Time t')
ylabel('y(t)')

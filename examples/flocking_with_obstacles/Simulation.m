% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

addpath(genpath('../../lib'))

% Clear workspace to increase repeatability
clear

%% Network parameters
agentCount = 50;  % Number of agents in the network
dimension  = 2;    % Dimension of the space the agents move in
dT         = 0.2; % Size of the simulation time steps [s]
Tf         = 50;   % Simulation time [s]

%% Initialize the network
config                  = SinrConfiguration();
config.agentCount       = agentCount;
config.slotCount        = 4;
config.cycleTime        = dT;
config.wirelessProtocol = WirelessProtocol.underwater_mako_modem;
config.power            = 5e-2;
config.packetSize       = 4*16;
config.pathLoss         = 2.3;
Network = SinrNetwork(config);

%% Place obstacles
obstacles = struct('center', {[75; 10], [75; 50]}, 'radius', {10, 10});

%% Initialize the agents
% To avoid Matlab initializing the array of agents without including the
% required constructor arguments, we first construct a cell array of agents
% and later convert this to a standard Matlab array.
Agents = cell(agentCount, 1);
for i = 1:length(Agents)
    % Randomly place the agents in the square [0, 50]^2
    pos = 25 + 50 * (rand(dimension, 1) - 0.5);
    vel = zeros(dimension, 1);
    
    % Initiallize an agent with the generated initial conditions
    Agents{i} = ObstacleAvoidingAgent(Network.getId(), dT, pos, vel, obstacles);
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
lap_history = zeros(floor(Tf/dT) + 1, agentCount, agentCount);

% Initialize remaining values for the simulation
t = 0;
k = 1;

tic
while t < Tf
    t = sim.step();
    
    % Save current position of all agents
    leech.save(t);

    if ~isempty(sim.lastLaplacian)
        lap_history(k,:,:) = sim.lastLaplacian;
        sim.lastLaplacian  = [];
        k = k + 1;
    end
end
fprintf("Simulation completed in %.3g seconds!\n", toc);

[~, ~, distinctCollisions] = checkCollisions(leech.data.position, 0.5);
fprintf("%d distinct collisions occurred!\n", distinctCollisions);

% Due to the multi-rate support, the sampling will not always be uniform.
% Therefore, we need to resample the data. The function uses a ZOH
% resampling approach. The resulting figure will be drawn with 20 FPS 
[t_sampled, sampled] = leech.resample(1/10);

%% Animate simulation results

% Compute boundary
x_pos  = squeeze(leech.data.position(:,1,:));
y_pos  = squeeze(leech.data.position(:,2,:));
bounds = @(x) [min(min(x)), max(max(x))];

% Compute resampled collisions
[filter, collisionCount] = checkCollisions(sampled.position, 0.5);

figure()
for k = 1:length(t_sampled)
    % Draw agents
    pos = squeeze(sampled.position(k,:,:));
    
    mask = filter(k,:);
    scatter(pos(1,~mask), pos(2,~mask), 'k')
    hold on
    scatter(pos(1,mask), pos(2,mask), 'r', 'filled')
    hold off
    
    % Draw obstacles
    hold on
    for obst = obstacles
        drawCircle(obst.center, obst.radius, true);
    end
    hold off
    
    % Limit viewpoint
    xlim(bounds(x_pos))
    ylim(bounds(y_pos))
    drawnow limitrate
end

figure()
meanL = movmean(lap_history, 10, 1);
for i = 1:size(meanL, 1)
    imshow(-squeeze(meanL(i,:,:)), 'InitialMagnification', 'fit')
    drawnow limitrate
end

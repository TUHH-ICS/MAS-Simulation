% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

addpath(genpath('../../lib'))
addpath('functions')

% Clear workspace to increase repeatability
clear

% Reset the Matlab profiler
profile clear

% Seed the pseudo random number generator. This is required if we want
% reproducible simulation, e. g. for profiling the code.
rng(0);

% Flag to enable exporting a video from the simulation results
saveVideo = false;

%% Network parameters
agentCount = 50;  % Number of agents in the network
dimension  = 2;    % Dimension of the space the agents move in
dT         = 0.2; % Size of the simulation time steps [s]
Tf         = 50;   % Simulation time [s]

% Type of communication, 1->ideal, 2->Bernoulli, 3->SINR
netType    = 3;

%% Initialize the network
switch netType
    case 1
        range   = 10;     % Range of the radio communication
        Network = IdealNetwork(agentCount, dT, dimension, range);
    case 2
        range     = Inf;   % Range of the radio communication
        pTransmit = 0.5;  % Probability of successful transmission
        Network   = BernoulliNetwork(agentCount, dT, dimension, range, pTransmit);
    case 3
        config                  = SinrConfiguration();
        config.agentCount       = agentCount;
        config.slotCount        = 4;
        config.cycleTime        = dT;
        config.wirelessProtocol = WirelessProtocol.underwater_mako_modem;
        config.power            = 5e-2;
        config.packetSize       = 4*16;
        config.pathLoss         = 2.3;
        enableSubstepping       = false; % If true, the messages will be distributed among the agents according to the slot timing
        Network = SinrNetwork(config, enableSubstepping);
        
        fprintf('The datarate of the network is %d bits/s\n', config.packetSize / config.slotTime);
end

%% Place obstacles
obstacles = struct('center', {[75; 10], [75; 50]}, 'radius', {10, 10});

%% Initialize the agents
% To avoid Matlab initializing the array of agents without including the
% required constructor arguments, we first construct a cell array of agents
% and later convert this to a standard Matlab array.
Agents = cell(agentCount, 1);
for i = 1:length(Agents)
    % Randomly place the agents in the square [0, 120]^2
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
k = 0;
k_lap = 1;

% Save start time of the simulation. We want to periodically print the
% progress of the simulation.
lastprint = posixtime(datetime('now'));

tic
% profile on
while t < Tf
    t = sim.step();
    k = k + 1;
    
    % Save current position of all agents
    leech.save(t);

    if ~isempty(sim.lastLaplacian)
        lap_history(k_lap,:,:) = sim.lastLaplacian;
        sim.lastLaplacian = [];
        k_lap = k_lap + 1;
    end
    
    % Print progress every 2 seconds
    if posixtime(datetime('now')) - lastprint >= 1
        lastprint = posixtime(datetime('now'));
        fprintf('Simulation %3.5g%% finished\n', 100*t/Tf)
    end
end
% profile viewer
fprintf("Simulation completed in %.3g seconds!\n", toc);

[~, ~, distinctCollisions] = checkCollisions(leech.data.position, 0.5);
fprintf("%d distinct collisions occurred!\n", distinctCollisions);

%% Resample data for plotting
% Due to the multi-rate support, the sampling will not always be uniform.
% Therefore, we need to resample the data. The resampling is oriented on
% the set parameters of the video, that may be produced

TVideo = 20; % Desired duration of the video [s]
FPS    = 30; % Framerate of the video [Hz]

% Calculate the required sampling time to meet the expectations
dTAnimate = Tf / (TVideo * FPS);

% Calculate the required sampling time to meet the expectations
dTAnimate = Tf / (TVideo * FPS);
[t_sampled, sampled] = leech.resample(dTAnimate);

%% Animate simulation results

% Compute boundary
x_pos  = squeeze(leech.data.position(:,1,:));
y_pos  = squeeze(leech.data.position(:,2,:));
bounds = @(x) [min(min(x)), max(max(x))];

% Compute resampled collisions
[filter, collisionCount] = checkCollisions(sampled.position, 0.5);

% Open video file with mp4 encoding
if saveVideo
    video = VideoWriter('obstacles', 'MPEG-4');
    video.FrameRate = FPS;
    open(video)
else
    video = [];
end

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
    
    if ~isempty(video)
        % Save figure as video frame
        frame = getframe(gcf);
        writeVideo(video, frame);
    end
end

if ~isempty(video)
    close(video)
    video = [];
end

meanL = movmean(lap_history, 10, 1);

figure()
for i = 1:size(meanL, 1)
    imshow(-squeeze(meanL(i,:,:)), 'InitialMagnification', 'fit')
    drawnow limitrate
end
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

% Flag to enable exporting a video from the simulation results
saveVideo = false;

%% Network parameters
agentCount = 20;   % Number of agents in the network
neighbours = 3;    % Number of neighbours for each agent
dimension  = 2;    % Dimension of the space the agents move in
dT         = 1e-2; % Size of the simulation time steps [s]
Tf         = 3;  % Simulation time [s]

% Type of communication, 1->ideal, 2->Bernoulli, 3->SINR
netType    = 1;

%% Initialize the network
switch netType
    case 1
        range   = Inf;   % Range of the radio communication
        Network = IdealNetwork(agentCount, dT, dimension, range);
    case 2
        range     = Inf; % Range of the radio communication
        pTransmit = 0.1; % Probability of successful transmission
        Network   = BernoulliNetwork(agentCount, dT, dimension, range, pTransmit);
    case 3
        config                  = SinrConfiguration();
        config.agentCount       = agentCount;
        config.slotCount        = agentCount;
        config.cycleTime        = dT;
        config.wirelessProtocol = WirelessProtocol.wp_802_11p;
        config.power            = 500e-3;
        config.packetSize       = 3*64;
        enableSubstepping       = false; % If true, the messages will be distributed among the agents according to the slot timing
        Network = SinrNetwork(config, enableSubstepping);
end

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

%% Resample data for plotting
% Due to the multi-rate support, the sampling will not always be uniform.
% Therefore, we need to resample the data. The resampling is oriented on
% the set parameters of the video, that may be produced

TVideo = 10; % Desired duration of the video [s]
FPS    = 30; % Framerate of the video [Hz]

% Calculate the required sampling time to meet the expectations
dTAnimate = Tf / (TVideo * FPS);

% Resample the data. The function uses a ZOH resampling approach 
[t_sampled, sampled] = leech.resample(dTAnimate);

%% Animate simulation results
figure()

% Compute boundary
x_pos  = squeeze(leech.data.position(:,1,:));
y_pos  = squeeze(leech.data.position(:,2,:));
bounds = @(x) [min(min(x)), max(max(x))];

% Open video file with mp4 encoding
if saveVideo
    video = VideoWriter('formation', 'MPEG-4');
    video.FrameRate = FPS;
    open(video)
else
    video = [];
end

for k = 1:length(t_sampled)
    pos   = squeeze(sampled.position(k,:,:));
    scatter(pos(1,:), pos(2,:))
    xlabel('x(t)')
    ylabel('y(t)')
    
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
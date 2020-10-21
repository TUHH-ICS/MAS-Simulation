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
agentCount = 50;   % Number of agents in the network
dimension  = 2;    % Dimension of the space the agents move in
dT         = 0.11;  % Size of the simulation time steps [s]
Tf         = 100;  % Simulation time [s]

% Type of communication, 1->ideal, 2->Bernoulli, 3->SINR
netType    = 3;

%% Initialize the network
switch netType
    case 1
        range   = 10;      % Range of the radio communication
        Network = IdealNetwork(agentCount, dT, dimension, range);
    case 2
        range     = 10;    % Range of the radio communication
        pTransmit = 0.95; % Probability of successful transmission
        Network   = BernoulliNetwork(agentCount, dT, dimension, range, pTransmit);
    case 3
        config                  = SinrConfiguration();
        config.agentCount       = agentCount;
        config.slotCount        = agentCount;
        config.cycleTime        = dT;
        config.wirelessProtocol = WirelessProtocol.wp_802_11p;
        config.power            = 3e-5;
        config.pathLoss         = 2.5;
        config.packetSize       = 4*64;
        enableSubstepping       = false; % If true, the messages will be distributed among the agents according to the slot timing
        Network = SinrNetwork(config, enableSubstepping);
end

%% Initialize the agents
% To avoid Matlab initializing the array of agents without including the
% required constructor arguments, we first construct a cell array of agents
% and later convert this to a standard Matlab array.
Agents = cell(agentCount, 1);
for i = 1:length(Agents)
    % Randomly place the agents in the square [0,100]^2
    pos = 50 + 100 * (rand(dimension, 1) - 0.5);
    
    % Point the agents to the center of the square at the start. In this
    % way, the agents will not fragment into multiple groups. This is not
    % required if a gamma agent would be included
    vel = 0.01 * ([ 50; 50 ] - pos);
    
    % Initiallize an agent with the generated initial conditions
    Agents{i} = FlockingAgent(Network.getId(), dT, pos, vel);
end
Agents = [Agents{:}];

%% Perform simulation
% The main work of the simulation is performed by the SimulationManager. It
% calls the agent and network routines in the desired frequency
sim   = SimulationManager(Network, Agents);

% Estimate the number of steps based on the final simulation time
steps = sim.estimateSteps(Tf);

% Preallocate storage for simulation results
leech = DataLeech(Agents, steps, 'position', 'velocity');

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

TVideo = 20; % Desired duration of the video [s]
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
    video = VideoWriter('flocking', 'MPEG-4');
    video.FrameRate = FPS;
    open(video)
else
    video = [];
end

for k = 1:length(t_sampled)
    pos = squeeze(sampled.position(k,:,:));
    scatter(pos(1,:), pos(2,:))
    
    xlim(bounds(x_pos))
    ylim(bounds(y_pos))
    title('Movement of the Agents')
    xlabel('Coordinate x(t)')
    ylabel('Coordinate y(t)')
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

%% Post processesing to calculate energy of the network
Ekin = zeros(size(t_sampled));
Epot = zeros(size(t_sampled));

% Calculate a sample of the Olfati-Saber interaction field. Because there
% is no analytic expression for the potential, we need to numerically
% integrate the force. This is however quite slow, thus we sample in
% advance and use an approximation for the field.
z_sample   = linspace(0, FlockingAgent.ra, 100); % Grid argument space
psi_sample = psi_alpha(z_sample, FlockingAgent.ra, FlockingAgent.da, FlockingAgent.ha);

figure()
plot(z_sample, psi_sample - min(psi_sample))
title('Shape of the Potential Field')
xlabel('Distance z')
ylabel('Potential \Psi_\alpha(z)')

% Calculate the energy at each agent
for k = 1:length(Ekin)
    pos = squeeze(sampled.position(k,:,:));
    vel = squeeze(sampled.velocity(k,:,:));
    
    Ekin(k) = sum(vecnorm(vel).^2) / 2;
    for i = 1:agentCount
        diff = pos - pos(:,i); % Arrow to all agents from agent i
        diff(:,i) = []; % Remove pointer to itself
        dist = vecnorm(diff); % Calculate distance from arrows
        dist = sigma_norm(dist, FlockingAgent.epsilon); % Convert to sigma norm
        
        % Calculate interaction field for this agent. First we calculate
        % the sampled bins the distances fall into.
        bins = sum(dist >= z_sample');
        % And after that we sum over all contributions.
        Epot(k) = Epot(k) + sum(psi_sample(bins));
    end
end

% Normalize the potential energy s.t. min(Epot) = 0
Epot = Epot - min(Epot);

figure()
plot(t_sampled, Ekin, t_sampled, Epot, t_sampled, Ekin + Epot)
title('Energy distribution')
xlabel('Time t')
ylabel('Energy E(t)')
legend('Kinetic Energy', 'Potential Energy', 'Total Energy')

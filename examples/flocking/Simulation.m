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
dT         = 0.1;  % Size of the simulation time steps [s]
Tf         = 100;  % Simulation time [s]

% Type of communication, 1->ideal, 2->Bernoulli, 3->SINR
netType    = 3;

%% Initialize the network
switch netType
    case 1
        range   = 6;      % Range of the radio communication
        Network = IdealNetwork(agentCount, dT, dimension, range);
    case 2
        range     = 6;    % Range of the radio communication
        pTransmit = 0.95; % Probability of successful transmission
        Network   = BernoulliNetwork(agentCount, dT, dimension, range, pTransmit);
    case 3
        config                  = SinrConfiguration();
        config.agentCount       = agentCount;
        config.slotCount        = agentCount;
        config.cycleTime        = dT;
        config.wirelessProtocol = WirelessProtocol.wp_802_11p;
        config.power            = 500e-3;
        config.packetSize       = 6*64;
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
t_history   = zeros(steps, 1);
pos_history = zeros(steps, dimension, agentCount);

% Initialize remaining values for the simulation
t = 0;
k = 0;

% Save start time of the simulation. We want to periodically print the
% progress of the simulation.
lastprint = posixtime(datetime('now'));

tic
% profile on
while t < Tf
    t = sim.step();
    k = k + 1;
    
    % Save current position of all agents
    t_history(k)       = t;
    pos_history(k,:,:) = [Agents.position];

    % Print progress every 2 seconds
    if posixtime(datetime('now')) - lastprint >= 1
        lastprint = posixtime(datetime('now'));
        fprintf('Simulation %3.5g%% finished\n', 100*t/Tf)
    end
end
% profile viewer
fprintf("Simulation completed in %.3g seconds!\n", toc);

%% Animate simulation results
figure()

% Compute boundary
x_pos  = squeeze(pos_history(:,1,:));
y_pos  = squeeze(pos_history(:,2,:));
bounds = @(x) [min(min(x)), max(max(x))];

% Open video file with mp4 encoding
if saveVideo
    video = VideoWriter('flocking', 'MPEG-4');
    video.FrameRate = 50;
    open(video)
else
    video = [];
end

for k = 1:steps
    pos = squeeze(pos_history(k,:,:));
    scatter(pos(1,:), pos(2,:))
    
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
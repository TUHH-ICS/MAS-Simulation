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
agentCount = 5;    % Number of agents in the network
dimension  = 2;    % Dimension of the space the agents move in
dT         = 0.01; % Size of the simulation time steps
Tf         = 200;    % Final simulation time

% Type of communication, 1->ideal, 2->Bernoulli, 3->SINR
netType    = 3;

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
        config.power            = 50e-3;
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
    if i == 1
        ref = [0; 0];
        pos = [0; 0];
    else
        phi = (i - 1.5)/(agentCount - 1) * 2*pi;
        ref = [cos(phi); sin(phi)];
        pos = 10 * (rand(dimension, 1) - 0.5);
    end
    
    Agents{i} = FormationUnicycle(Network.getId(), pos, ref);
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
cns_history = zeros(steps, dimension, agentCount);
phi_history = zeros(steps, agentCount);

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
    cns_history(k,:,:) = [Agents.consens];
    states             = [Agents.state];
    phi_history(k,:)   = states(4,:);

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
ts_pos  = timeseries(permute(pos_history, [2 3 1]), t_history, 'Name', 'position');
ts_cns  = timeseries(permute(cns_history, [2 3 1]), t_history, 'Name', 'consensus');
ts_phi  = timeseries(phi_history', t_history, 'Name', 'phi');
ts = tscollection({ts_pos, ts_cns, ts_phi});
tsout = resample(ts, 0:dTAnimate:Tf, 'zoh');

% Extract the resampled data
pos_resampled = permute(tsout.position.Data, [3 1 2]);
cns_resampled = permute(tsout.consensus.Data, [3 1 2]);
phi_resampled = squeeze(tsout.phi.Data)';
t_resampled   = tsout.position.Time;
step_sampled  = tsout.position.length;

%% Animate simulation results
figure()

% Compute boundary
x_pos  = squeeze(pos_history(:,1,:));
x_cns  = squeeze(cns_history(:,1,:));
y_pos  = squeeze(pos_history(:,2,:));
y_cns  = squeeze(cns_history(:,2,:));
bounds = @(x) [min(min(x)), max(max(x))];

x_bounds = bounds([x_pos; x_cns]);
y_bounds = bounds([y_pos; y_cns]);

% Open video file with mp4 encoding
if saveVideo
    video = VideoWriter('formation', 'MPEG-4');
    video.FrameRate = FPS;
    open(video)
else
    video = [];
end

rad = linspace(0, 2*pi);
ref = [4*sin(2*rad); 8*sin(rad)];

for k = 1:step_sampled
    pos = squeeze(pos_resampled(k,:,:));
    cns = squeeze(cns_resampled(k,:,:));
    phi = phi_resampled(k,:);
    
    cycledir = [cos(phi); sin(phi)];
    cg = pos - [Agents.d] .* cycledir;
    front = cg + 1.25 * [Agents.d] .* cycledir;
    back  = cg - 1.25 * [Agents.d] .* cycledir;
    
    plot(ref(1,:), ref(2,:), '--');
    hold on
    line([front(1,:); back(1,:)], [front(2,:); back(2,:)], 'Color', 'k', 'LineWidth', 5)
    scatter(cg(1,:), cg(2,:), 'y', 'filled')
    scatter(pos(1,:), pos(2,:), 'r', 'filled')
    scatter(cns(1,:), cns(2,:), 'b+')
    hold off
    xlabel('x(t)')
    ylabel('y(t)')
    
    xlim(x_bounds)
    ylim(y_bounds)
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

t_bounds = bounds(t_history);

figure()
subplot(211)
plot(t_history, x_cns, '--', t_history, x_pos, '-')
xlim(t_bounds);
xlabel('Time t')
ylabel('x(t)')

subplot(212)
plot(t_history, y_cns, '--', t_history, y_pos, '-')
xlim(t_bounds);
xlabel('Time t')
ylabel('y(t)')
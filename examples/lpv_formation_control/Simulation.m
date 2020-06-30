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
Tf         = 100;    % Final simulation time
range      = Inf;  % Range of the radio communication
pTransmit  = 0.95; % Probability of successful transmission

%% Initialize the network
Network = IdealNetwork(agentCount, dT, dimension, range);
% Network = BernoulliNetwork(agentCount, dT, dimension, range, pTransmit);

% To avoid Matlab initializing the array of agents without including the
% required constructor arguments, we first construct a cell array of agents
% and later convert this to a standard Matlab array.
Agents = cell(agentCount, 1);
for i = 1:length(Agents)
    Agents{i} = FormationUnicycle(Network.getId(), zeros(dimension,1), [i^2; 0]);
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
ts = tscollection({ts_pos, ts_cns});
tsout = resample(ts, 0:dTAnimate:Tf, 'zoh');

% Extract the resampled data
pos_resampled = permute(tsout.position.Data, [3 1 2]);
cns_resampled = permute(tsout.consensus.Data, [3 1 2]);
t_resampled   = tsout.position.Time;
step_sampled  = tsout.position.length;

%% Animate simulation results
figure()

% Compute boundary
x_pos  = squeeze(pos_resampled(:,1,:));
x_cns  = squeeze(cns_resampled(:,1,:));
y_pos  = squeeze(pos_resampled(:,2,:));
y_cns  = squeeze(cns_resampled(:,2,:));
bounds = @(x) [min(min(x)), max(max(x))];

x_bounds = bounds([x_pos; x_cns]);
y_bounds = bounds([y_pos; y_cns]);

% Open video file with mp4 encoding
if saveVideo
    video = VideoWriter('formation', 'MPEG-4');
    video.FrameRate = 50;
    open(video)
else
    video = [];
end

for k = 1:step_sampled
    pos = squeeze(pos_resampled(k,:,:));
    cns = squeeze(cns_resampled(k,:,:));
    
    scatter(cns(1,:), cns(2,:), '+')
    hold on
    scatter(pos(1,:), pos(2,:))
    hold off
    xlabel('x(t)')
    ylabel('y(t)')
    zlabel('z(t)')
    
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

figure()
subplot(211)
plot(t_resampled, x_cns, '--', t_resampled, x_pos, '-')
xlabel('Time t')
ylabel('x(t)')

subplot(212)
plot(t_resampled, y_cns, '--', t_resampled, y_pos, '-')
xlabel('Time t')
ylabel('y(t)')
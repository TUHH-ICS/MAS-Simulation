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
dimension  = 3;    % Dimension of the space the agents move in
dT         = 0.01; % Size of the simulation time steps
range      = 150;  % Range of the radio communication
pTransmit  = 0.95; % Probability of successful transmission
steps      = 500;  % Simulation time steps

%% Initialize the network
Network = IdealNetwork(agentCount, dimension, range);
% Network = BernoulliNetwork(agentCount, dimension, range, pTransmit);

% To avoid Matlab initializing the array of agents without including the
% required constructor arguments, we first construct a cell array of agents
% and later convert this to a standard Matlab array.
Agents = cell(agentCount, 1);
for i = 1:length(Agents)
    % Randomly place the agents in the square [0,30]^3
    pos = 15 + 30 * (rand(dimension, 1) - 0.5);
    
    % Initialize an agent with the generated initial conditions
%     Agents{i} = FormationAgent(Network, dT, pos, [i^2; 0; 0]);
    Agents{i} = FormationQuadrotor(Network, dT, pos, [i^2; 0; 0]);
end
Agents = [Agents{:}];

%% Perform simulation
pos_history = zeros(steps+1, dimension, agentCount);
pos_history(1,:,:) = [Agents.position];

tic
% profile on
for k = 1:steps
    % Perform a single time step for each agent. This includes evaluating
    % the dynamic equations and the flocking protocol as well as sending
    % its own position and velocity to the other agents as a message.
    for agent = Agents
        agent.step()
    end
    
    % Distribute messages among the agents
    Network.process()
    
    % Save current position of all agents
    pos_history(k+1,:,:) = [Agents.position];
    
    % Print status periodically
    if rem(k, steps/100) == 0
        fprintf('Simulation %3.5g%% finished\n', 100*k/steps)
    end
end
% profile viewer
toc

%% Animate simulation results
figure()

% Compute boundary
x_pos  = squeeze(pos_history(:,1,:));
y_pos  = squeeze(pos_history(:,2,:));
z_pos  = squeeze(pos_history(:,3,:));
bounds = @(x) [min(min(x)), max(max(x))];

% Open video file with mp4 encoding
if saveVideo
    video = VideoWriter('formation', 'MPEG-4');
    video.FrameRate = 50;
    open(video)
else
    video = [];
end

for k = 1:steps+1
    pos = squeeze(pos_history(k,:,:));
    scatter3(pos(1,:), pos(2,:), pos(3,:))
    xlabel('x(t)')
    ylabel('y(t)')
    zlabel('z(t)')
    
    xlim(bounds(x_pos))
    ylim(bounds(y_pos))
    zlim(bounds(z_pos))
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
subplot(311)
plot(dT*(0:steps), x_pos)
xlabel('Time t')
ylabel('x(t)')

subplot(312)
plot(dT*(0:steps), y_pos)
xlabel('Time t')
ylabel('y(t)')

subplot(313)
plot(dT*(0:steps), z_pos)
xlabel('Time t')
ylabel('z(t)')
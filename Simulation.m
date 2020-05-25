addpath(genpath('.'))
clear
profile clear

% Seed the pseudo random number generator. This is required if we want
% reproducible simulation, e. g. for profiling the code.
rng(0);

%% Network parameters
agentCount = 100;   % Number of agents in the network
dimension  = 2;     % Dimension of the space the agents move in
range      = 6;     % Range of the radio communication
steps      = 2000; % Simulation time steps

%% Initialize the network
Network = IdealNetwork(agentCount, dimension, range);

% To avoid Matlab initializing the array of agents without including the
% required constructor arguments, we first construct a cell array of agents
% and later convert this to a standard Matlab array.
Agents  = cell(agentCount, 1);
for i = 1:length(Agents)
    pos = 50 + 100 * (rand(dimension, 1) - 0.5);
    vel = 0.01 * ([ 50; 50 ] - pos);
    Agents{i} = FlockingAgent(Network, pos, vel);
end
Agents  = [Agents{:}];

%% Perform simulation
pos_history = zeros(steps+1, dimension, agentCount);
pos_history(1,:,:) = [Agents.position];

tic
profile on
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
profile viewer
toc

%% Animate simulation results
figure()

% Compute boundary
x_pos = squeeze(pos_history(:,1,:));
y_pos = squeeze(pos_history(:,2,:));
bounds = @(x) [min(min(x)), max(max(x))];

for k = 1:steps+1
    pos = squeeze(pos_history(k,:,:));
    scatter(pos(1,:), pos(2,:))
    xlim(bounds(x_pos));
    ylim(bounds(y_pos));
    drawnow limitrate
end
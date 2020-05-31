addpath(genpath('../../lib'))
addpath('functions')

% Clear workspace to increase repeatability
clear

% Reset the Matlab profiler
profile clear

% Seed the pseudo random number generator. This is required if we want
% reproducible simulation, e. g. for profiling the code.
rng(0);

%% Network parameters
agentCount = 50;   % Number of agents in the network
dimension  = 2;    % Dimension of the space the agents move in
dT         = 0.1;  % Size of the simulation time steps
range      = 7*1.2;% Range of the radio communication
pTransmit  = 1;    % Probability of successful transmission
steps      = 1000; % Simulation time steps

%% Source seeking parameters
source_seek_params=source_seeking_parameters();
field=field_inv_gaussians(source_seek_params);
%% Initialize the network
% Network = IdealNetwork(agentCount, dimension, range);
Network = BernoulliNetwork(agentCount, dimension, range, pTransmit);

% To avoid Matlab initializing the array of agents without including the
% required constructor arguments, we first construct a cell array of agents
% and later convert this to a standard Matlab array.
Agents  = cell(agentCount, 1);
for i = 1:length(Agents)
    % Randomly place the agents in the square [0,100]^2
    pos = 50 + 100 * (rand(dimension, 1) - 0.5);
    % Randomly assign the agent velocites in [-5,5]^2
    vel = 10 * (rand(dimension, 1) - 0.5);
    
    % Initiallize an agent with the generated initial conditions
    Agents{i} = FlockingAgent2(Network, dT, pos, vel);
end
Agents  = [Agents{:}];

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
        grad_true=field.get_true_gradient(agent.position);
        agent.get_gradient_est(grad_true);
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
x_pos = squeeze(pos_history(:,1,:));
y_pos = squeeze(pos_history(:,2,:));
bounds = @(x) [min(min(x)), max(max(x))];
lim=[bounds(x_pos);bounds(y_pos)];
[X,Y,Z]=data_for_contour(lim,source_seek_params);
for k = 1:steps+1
    contour(X,Y,Z)
    hold on
    pos = squeeze(pos_history(k,:,:));
    scatter(pos(1,:), pos(2,:))
    hold off
    xlim(bounds(x_pos));
    ylim(bounds(y_pos));     
    drawnow limitrate
end

  

addpath(genpath('.'))
clear

%% Network parameters
agentCount = 10; % Number of agents in the network
dimension  = 2;  % Dimension of the space the agents move in
range      = 10; % Range of the radio communication

%% Initialize the network
Network    = IdealNetwork(agentCount, dimension, range);

Agents     = cell(agentCount, 1);
for i = 1:length(Agents)
    Agents{i} = FlockingAgent(Network, rand(dimension, 1));
end
Agents     = [Agents{:}];

%% Perform simulation

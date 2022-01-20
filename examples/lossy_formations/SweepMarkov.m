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

%% Network parameters
agentCount = 30;    % Number of agents in the network
dT         = 0.01; % Size of the simulation time steps [s]
Tf         = 5;    % Simulation time [s]

% Network settings
range      = 3;    % Communication range
symmetric  = true; % Symmetric or asymmetric link failure

pfail = linspace(0,1,10);
preco = linspace(0,1,10);
n = 1:5;
[PFail, PReco, ~] = meshgrid(pfail, preco, n);
rmse   = zeros(size(PFail));

parfor i = 1:length(pfail)*length(preco)*length(n)
    Network = MarkovNetwork(agentCount, dT, 3, range, PFail(i), PReco(i), [], symmetric);    

    %% Initialize the agents
    % To avoid Matlab initializing the array of agents without including the
    % required constructor arguments, we first construct a cell array of agents
    % and later convert this to a standard Matlab array.
    Agents = cell(agentCount, 1);
    for j = 1:length(Agents)
        % Randomly place the agents in the square [0,30]^3
        pos = 15 + 30 * (rand(3, 1) - 0.5);

        % Initialize an agent with the generated initial conditions
        Agents{j} = FormationQuadrotor(Network.getId(), pos, [j; 0; 0]);
    end
    Agents = [Agents{:}];

    %% Perform simulation
    % The main work of the simulation is performed by the SimulationManager. It
    % calls the agent and network routines in the desired frequency
    sim   = SimulationManager(Network, Agents);
    steps = sim.estimateSteps(Tf);

    % Preallocate storage for simulation results
    leech = DataLeech(Agents, steps, 'position', 'ref');

    % Simulate!
    t = 0;
    while t < Tf
        t = sim.step();
        leech.save(t)
    end

    %% Calculate mean square formation error    
    pos = leech.data.position(:);
    ref = leech.data.ref(:);
    rmse(i) = sqrt(sum((pos - ref).^2));
    
    fprintf('Iteration %d done!\n', i)
end

rmse_mean = mean(rmse, 3);

%%
figure()
surf(PFail(:,:,1), PReco(:,:,1), rmse_mean)
xlabel('Probability p_{fail}')
ylabel('Probability p_{reco}')
zlabel('Mean RMSE(p)')

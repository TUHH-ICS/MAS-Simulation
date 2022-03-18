%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

addpath(genpath('../../lib'))

% Clear workspace to increase repeatability
clear

%% Network parameters
agentCount = 5;    % Number of agents in the network
dimension  = 2;    % Dimension of the space the agents move in
dT         = 0.01; % Size of the simulation time steps
Tf         = 200;    % Final simulation time

%% Initialize the network
config                  = SinrConfiguration();
config.agentCount       = agentCount;
config.slotCount        = agentCount;
config.cycleTime        = dT;
config.wirelessProtocol = WirelessProtocol.wp_802_11p;
config.power            = 50e-3;
config.packetSize       = 3*64;
Network = SinrNetwork(config);

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
leech = DataLeech(Agents, steps, 'position', 'consens', 'state');

% Save start time of the simulation. We want to periodically print the
% progress of the simulation.
lastprint = posixtime(datetime('now'));

% Simulate!
tic
t = 0;
while t < Tf
    t = sim.step();
    
    % Save current position of all agents
    leech.save(t);

    % Print progress every 2 seconds
    if posixtime(datetime('now')) - lastprint >= 1
        lastprint = posixtime(datetime('now'));
        fprintf('Simulation %3.5g%% finished\n', 100*t/Tf)
    end
end
fprintf("Simulation completed in %.3g seconds!\n", toc);

% Due to the multi-rate support, the sampling will not always be uniform.
% Therefore, we need to resample the data. The function uses a ZOH
% resampling approach. The resulting figure will be drawn with 20 FPS 
[t_sampled, sampled] = leech.resample(1/2);

%% Animate simulation results
figure()

% Compute boundary
x_pos  = squeeze(leech.data.position(:,1,:));
x_cns  = squeeze(leech.data.consens(:,1,:));
y_pos  = squeeze(leech.data.position(:,2,:));
y_cns  = squeeze(leech.data.consens(:,2,:));
bounds = @(x) [min(min(x)), max(max(x))];

x_bounds = bounds([x_pos; x_cns]);
y_bounds = bounds([y_pos; y_cns]);

rad = linspace(0, 2*pi);
ref = [4*sin(2*rad); 8*sin(rad)];

for k = 1:length(t_sampled)
    pos = squeeze(sampled.position(k,:,:));
    cns = squeeze(sampled.consens(k,:,:));
    phi = squeeze(sampled.state(k,4,:))';
    
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
end

t_bounds = bounds(leech.t);

figure()
subplot(211)
plot(leech.t, x_cns, '--', leech.t, x_pos, '-')
xlim(t_bounds);
xlabel('Time t')
ylabel('x(t)')

subplot(212)
plot(leech.t, y_cns, '--', leech.t, y_pos, '-')
xlim(t_bounds);
xlabel('Time t')
ylabel('y(t)')

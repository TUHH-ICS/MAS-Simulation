% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

% The task of this script is to measure the properties of the communication
% channel provided by the SINR networking library. While the basics of the
% transmission are known, there is no intuition for the size and shape of
% the communication discs as well as the effect of the transmission power
% on these.
% Thus, we perform a Monte-Carlo like simulation to estimate the size of
% the communication discs.

addpath(genpath('../../lib'))

clear

%% Network configuration
% Configure the network with the desired properties. We only have two
% agents, of which only one will send messages, so one slot is sufficient.
% The main tuning knobs for the first figure are the transmission power and
% the path loss.
config                  = SinrConfiguration();
config.agentCount       = 2;
config.slotCount        = 1;
config.cycleTime        = 1;
config.wirelessProtocol = WirelessProtocol.wp_802_11p;
config.pathLoss         = 2.5;
config.power            = 3e-5;
config.packetSize       = 6*64;
Network = SinrNetwork(config);

%% Sweep over distance
% The first figure we are interested in is an estimate of the reception
% probability for different distances between the agents. Therefore, we
% perform a sweep over the distance and send a number of messages over the
% network to estimate the probability.
d = linspace(0, 30);
p = zeros(size(d));

tic
for i = 1:length(d)
    [sent, received] = sim_steps(Network, d(i), 10000);
    p(i) = received / sent;
end
fprintf("Simulation completed in %.3g seconds!\n", toc);

%% Plot resulting function
figure()
plot(d, p)
ylim([0, 1])

title(sprintf("Propability of reception with P = %g", config.power))
xlabel('Distance d')
ylabel('Probability p')

drawnow()

%% Sweep over power
% The second plot should show us the distance at which a certain reception
% probability is barely reached depending on the transmission power. For
% this, we perform a logarithmic sweep over a range of transmission powers.
%
% The search algorithm is then split into two phases. First, we search for
% an upper bound by exponentially increasing the distance between the
% sender and receiver for a given power until the desired reception
% probability is no longer reached. Afterwards, we apply a repeated
% bisection to calculate the desired distance to a given error threshold.
%
% In both phases, we transmit a number of messages over the channel to
% estimate the reception probability.

P = logspace(-8, -3, 100);
z = zeros(size(P));

threshold = 0.9;

tic
for i = 1:length(P)
    clear Network
    config.power = P(i);
    Network = SinrNetwork(config);
    
    % Initialize the search for a minumum distance of 1 mm.
    lb = 0.001;
    prop = 1;
   
    % First phase, exponential scaling to search for an upper bound
    while prop >= threshold
        ub = 5 * lb;
        [sent, received] = sim_steps(Network, ub, 1000);
        prop = received / sent;
        
        % As long as the threshold is not yet violated, we use the
        % simulated distance as the new lower bound.
        if prop >= threshold
            lb = ub;
        end
    end
    
    % Second phase, perform bisection
    while norm(ub - lb) > 1e-4 * lb
        mid = (ub + lb) / 2;
        [sent, received] = sim_steps(Network, mid, 1000);
        
        % Depending on whether we are currently above or below the
        % threshold, we adjust our lower or upper bounds on the threshold
        % distance.
        if received / sent >= threshold
            lb = mid;
        else
            ub = mid;
        end
    end
    
    % Save result
    z(i) = (ub + lb) / 2;
end
fprintf("Simulation completed in %.3g seconds!\n", toc);

%% Plot resulting function
figure()
semilogx(P, z)

title(sprintf("Distance with p ≈ %3g for different transmission powers", threshold))
xlabel('Transmission Power P')
ylabel('Distance d')

%% Helping function
function [sent, received] = sim_steps(Network, d, count)
    %SIM_STEPS Function that simulates the transmission of 'count' messages
    %over the given network for agents that are a distance of 'd' apart.
    %   This function returns the total number of messages sent and the
    %   number of messages that were received.

    % Initialize the agents
    tx = TX(1, 0, 1);
    rx = RX(2, d);
    Network.updateAgent(rx)

    % Send messages one after another
    for k = 1:count
        tx.step();
        Network.updateAgent(tx);

        % Transmit messages
        msg = Network.process();
        
        % Check if a messages was received at agent 2, the receiver
        if ~isempty(msg{2})
            rx.receive(msg{2});
        end
    end
    
    % Extract number of transmitted and received messages
    sent     = tx.MessageCount;
    received = rx.MessageCount;
end
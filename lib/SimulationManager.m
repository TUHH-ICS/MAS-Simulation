%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

classdef(Sealed) SimulationManager < handle
    %SIMULATIONMANAGER Class to manage the simulation of multi-rate
    %discrete-time multi-agent systems.
    %   This class makes it easier to simulate networks of agents with
    %   heterogeneous sample times among the agents and the network. The
    %   sample times do not need to be integer multiples.
    
    properties(GetAccess = public, SetAccess = public)
        lastLaplacian   % Last value of the networks Laplacian matrix
    end
    
    properties(GetAccess = public, SetAccess = private)
        nextAgentCall   % Contains the time when each agent should be called next
        nextNetworkCall % Contains the time when the network should be called next
        needsUpdate     % Flags for each agent, if it needs to be updated in the network
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        agents  % Array of agents, that should be simulated
        network % Network that the agents communicate over
    end

    methods
        function obj = SimulationManager(network, agents)
            %SIMULATIONMANAGER Construct an instance of this class
                       
            obj.agents  = agents;
            obj.network = network;
            
            % Initialize times and flags to the beginning of the simulation 
            agentCount = length(agents);
            obj.nextAgentCall   = zeros(agentCount, 1);
            obj.nextNetworkCall = 0;
            obj.needsUpdate     = ones(agentCount, 1, 'logical');
        end
        
        function k = estimateSteps(obj, T)
            %ESTIMATESTEPS Estimates the number of calls to the step
            %function that are required to reach the time T.
            %   The estimation of the number of calls to step() is required
            %   for preallocating the required buffer arrays. This
            %   function works by performing a dry run of the simulation,
            %   i. e. it stops the time but does not execute the agent
            %   functions.
            
            % Initialize temporary counters to start of simulation
            k     = 0;
            t     = 0;
            times = [obj.agents.dT, obj.network.cycleTime];
            calls = zeros(size(times));
            
            % Perform simulation steps until final time is reached
            while t < T
                k = k + 1;
                
                % Calculate time of next simulation step
                t = min(calls);
                
                % Calculate next step for updated agents
                mask = calls == t;
                calls(mask) = calls(mask) + times(mask);
            end
        end
        
        function t = step(obj)
            %STEP This method steps the simulation forward in time
            %   The time step that is performed by this method does not
            %   need to be uniform. It steps to the next "event", i. e.
            %   update of an agent or update of the network. This enables
            %   multi-rate simulation with non-integer multiples of the
            %   fundamental frequencies.
            
            % Calculate the time of the next action
            nextCall = min(obj.nextAgentCall);
            t = min(nextCall, obj.nextNetworkCall);

            % Check if the agents are next to run
            if nextCall <= obj.nextNetworkCall
                for agent = obj.agents(obj.nextAgentCall == nextCall)
                    agent.step()

                    % Set update required
                    obj.needsUpdate(agent.id) = true;

                    % Set time for next call to this agent
                    obj.nextAgentCall(agent.id) = t + agent.dT;
                end
            end

            % Check if the network is the next to run
            if obj.nextNetworkCall <= nextCall
                % If required, update the information that the network has
                % about the agents
                for agent = obj.agents(obj.needsUpdate)
                    obj.network.updateAgent(agent)
                end
                obj.needsUpdate(:) = false;

                % Process sent messages in the network
                recvMessages = obj.network.process();
                obj.lastLaplacian = calculateLaplacian(recvMessages);
                
                % Distribute transmitted messages among the agents
                mask = ~cellfun(@isempty, recvMessages);
                for agent = obj.agents(mask)
                    agent.receive(t, recvMessages{agent.id});
                end

                % Set time for next call
                obj.nextNetworkCall = t + obj.network.cycleTime;
            end
        end
    end
end

function L = calculateLaplacian(recvMessages)
    %CALCULATELAPLACIAN Function that calculates the Laplacian matrix based on
    %the received messages and the sender information contained in the messages.

    L = diag(cellfun(@length, recvMessages));
    for i = 1:length(recvMessages)
        L(i,[recvMessages{i}.sender]) = -1;
    end
end

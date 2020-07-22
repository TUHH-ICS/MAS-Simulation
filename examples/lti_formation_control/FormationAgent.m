% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef FormationAgent < LinearFrictionMass
    %FORMATIONAGENT Examplary agent implementation with simple LTI dynamics
    %that performs a formation control manoeuvre.
    
    % Define the constants of the agent and the consensus protocol
    properties(Constant)
        epsilon = 0.01; % Convergence speed of the consensus protocol
    end
    
    properties(GetAccess = public, SetAccess = protected)
        consens  % State of the consensus protocol
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        Fx       % State feedback coefficient
        Fr       % Feed-forward coefficient
        ref      % Formation reference
    end
    
    methods
        function obj = FormationAgent(id, dT, initialPos, reference)
            %FORMATIONAGENT Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent and consensus protocol to the given initial position.
            
            initialVel = zeros(size(initialPos));
            obj@LinearFrictionMass(id, dT, 1, 1, initialPos, initialVel);
           
            % Initialize the consensus protocol to the initial position of
            % the agent itself
            obj.consens = initialPos;
            
            % Set the formation reference if one is given to the agent
            if nargin <= 3
                obj.ref = zeros(size(initialPos));
            else
                obj.ref = reference;
            end
            
            % Set controller matrices
            obj.Fr = kron(eye(3),   168.8665);
            obj.Fx = kron(eye(3), [-168.8668, -25.0094]);
        end
        
        function step(obj)
            % Receive messages from the network
            messages = obj.receive();
            
            if ~isempty(messages)
                % Calculate new formation reference. We use the normalized
                % Laplacian, therefore we calculate the mean of the
                % positions, not the sum
                data        = [messages.data];
                positions   = [data.position];
                dist        = mean(obj.position - positions - obj.ref, 2);
                obj.consens = obj.consens - obj.epsilon * (dist);
            end
            
            % Evaluate agent dynamics
            u = obj.Fr * obj.consens + obj.Fx * obj.state;
            obj.move(u);
            
            % Send message to network, include only the position 
            data = struct;
            data.position = obj.position - obj.ref;
            obj.send(data)
        end
    end
end


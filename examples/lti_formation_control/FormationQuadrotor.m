%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

classdef FormationQuadrotor < LinearisedQuadrocopter
    %FORMATIONQUADROTOR Quadrotor agent that performs a formation control
    %maneuvre in a group of agents.
    
    properties(GetAccess = public, SetAccess = immutable)
        ref        % Formation reference
    end
    
    properties(GetAccess = private, SetAccess = immutable)
        controller % Discrete-time LTI controller
    end
    
    methods
        function obj = FormationQuadrotor(id, initialPos, reference)
            %FORMATIONQUADROTOR Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent and consensus protocol to the given initial position.
            
            data = load('quadrotor_model');
            
            % Initialize quadrotor model
            initialVel = zeros(size(initialPos));
            obj@LinearisedQuadrocopter(id, data.Ts, data.m, initialPos, initialVel);
            
            % Assemble controller
            obj.controller = DiscreteLtiDynamics(data.Ad, data.Bd, data.Cd, data.Dd);
                
            % Set the formation reference if one is given to the agent
            if nargin <= 2
                obj.ref = zeros(size(initialPos));
            else
                obj.ref = reference;
            end
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
                dist        = mean(positions, 2) + obj.ref - obj.position;
            else
                dist        = zeros(3,1);
            end
            
            % Evaluate agent dynamics
            e_hat = [dist; -obj.state];
            u     = obj.controller.step(e_hat);
            obj.move(u);
            
            % Send message to network, include only the position 
            data = struct;
            data.position = obj.position - obj.ref;
            obj.send(data)
        end
    end
end

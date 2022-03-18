%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

classdef FormationUnicycle < DynamicUnicycle
    %FORMATIONUNICYCLE Agent implementation with a dynamic unicycle model.
    %   These agents will perform a formation control maneuvre, where agent
    %   1 acts as a leader and will guide the formation along a pre-defined
    %   reference trajectory.
    
    properties(Constant)
        epsilon = 0.005; % Convergence speed of the consensus protocol
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        ref % Formation reference
    end
    
    properties(GetAccess = public, SetAccess = protected)
        consens  % State of the consensus protocol
        t = 0    % Current time of the simulation
    end
    
    properties(GetAccess = private, SetAccess = immutable)
        controller % Dynamic output-feedback controller
    end
    
    methods
        function obj = FormationUnicycle(id, initialPos, reference)
            %FORMATIONUNICYCLE Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent and consensus protocol to the given initial position.
            
            % Load data from synthesis script
            data = load('unicycle_controller');
            obj@DynamicUnicycle(id, data.Ts, data.d, data.m, data.Iz, initialPos);
            
            % Build controller
            A = @(rho) data.Ak0 + rho * data.Ak1;
            B = @(rho) data.Bk0 + rho * data.Bk1;
            C = @(rho) data.Ck0 + rho * data.Ck1;
            D = @(rho) data.Dk0 + rho * data.Dk1;
            obj.controller = DiscreteLpvDynamics([], A, B, C, D, zeros(8,1));
            
            % Initialize the consensus protocol to the initial position of
            % the agent itself
            obj.consens = initialPos;
            
            % Set the formation reference if one is given to the agent
            if nargin <= 2
                obj.ref = zeros(2,1);
            else
                obj.ref = reference;
            end
        end

        function step(obj)
            % Receive messages from the network
            messages = obj.receive();
            
            % This block implements a leader follower scheme. The agent
            % with id 1 ignores all incomming messages but instead follows
            % a pre-defined reference. It is thus the leader of the
            % formation.
            if obj.id == 1
                obj.consens = [4*sin(2*pi*obj.t/100)
                               8*sin(2*pi*obj.t/200)];
            else
                if ~isempty(messages)
                    % Calculate new formation reference. We use the normalized
                    % Laplacian, therefore we calculate the mean of the
                    % positions, not the sum
                    data        = [messages.data];
                    positions   = [data.position];
                    dist        = obj.consens - obj.ref - mean(positions, 2);
                    obj.consens = obj.consens - obj.epsilon * dist;
                end
            end
            
            % Extract state information from the unicycle model
            phi   = obj.state(4);
            omega = obj.state(5);

            % For the controller synthesis, we used a rotated frame of
            % reference for the agents. As these are state feedback
            % controllers, we need to also rotate the frame of reference
            % here to be consistent with the synthesis.
            rot = [ cos(phi), sin(phi); -sin(phi), cos(phi) ];

            % Calculate current scheduling parameter vt
            rho = obj.d * omega;
            
            % Calculate local tracking error of the unicycle
            e = obj.consens - obj.position;
            
            % Evaluate controller equation
            u = obj.controller.step(rot * e, rho);
            obj.move(u);
            obj.t = obj.t + obj.dT;
                        
            % Send message to network, include only the position 
            data = struct;
            data.position = obj.consens - obj.ref;
            obj.send(data)
        end
    end
end

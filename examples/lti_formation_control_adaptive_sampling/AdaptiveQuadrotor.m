% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef AdaptiveQuadrotor < LinearisedQuadrocopter
    %ADAPTIVEQUADROTOR Quadrotor agent that performs a formation control
    %maneuvre in a group of agents.
    
    properties(GetAccess = public, SetAccess = private)
        sample_factor % Current ratio of the base sampling time to the actual one (>= 1)
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        ref           % Formation reference
    end
    
    properties(GetAccess = private, SetAccess = private)
        skip_counter  % Counter of samples to be skipped till next controller update
        t = 0         % Current simulation time
    end
    
    properties(GetAccess = private, SetAccess = immutable)
        controller    % Discrete-time LTI controller
    end
    
    properties(Dependent, GetAccess = public, SetAccess = private)
        dT_effective
    end
    
    methods
        function obj = AdaptiveQuadrotor(id, initialPos, reference)
            %ADAPTIVEQUADROTOR Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent and consensus protocol to the given initial position.
            
            data = load('quadrotor_model');
            
            % Initialize quadrotor model
            initialVel = zeros(size(initialPos));
            obj@LinearisedQuadrocopter(id, data.Ts/10, data.m, initialPos, initialVel);
            
            % Assemble controller
            [Ad, Bd, Cd, Dd] = ssdata(data.K);
            obj.controller = DiscreteLtiDynamics(Ad, Bd, Cd, Dd);
                
            % Set the formation reference if one is given to the agent
            if nargin <= 2
                obj.ref = zeros(size(initialPos));
            else
                obj.ref = reference;
            end
            
            % Always sample in the first step
            obj.sample_factor = 1;
            obj.skip_counter  = 1; 
        end
        
        function value = get.dT_effective(obj)
            %GET.DT_EFFECTIVE Calculates the effective sampling time for
            %this agent.
            
            value = obj.dT * obj.sample_factor;
        end
        
        function step(obj)
            obj.skip_counter = obj.skip_counter - 1;
            
            if obj.skip_counter == 0
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
                
                % Calculate new sampling factor to reduce the bandwidth
                obj.sample_factor = round(10 + 5 * sin((obj.t + obj.id / 15) * 2 * pi));
                obj.skip_counter = obj.sample_factor;
            else
                obj.move();
            end
            
            obj.t = obj.t + obj.dT;
        end
    end
end


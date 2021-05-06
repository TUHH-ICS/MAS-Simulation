% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef DynamicUnicycleAgent < DynamicUnicycle
    %DYNAMICUNICYCLEAGENT Agent.    
    
    properties(GetAccess = private, SetAccess = immutable)
        controller % Discrete-time LTI controller
    end
   
    methods
        function obj = DynamicUnicycleAgent(id,initialPos)
            %DYNAMICUNICYCLEAGENT Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent to the given initial position.
            
            data = load('unicycle_controller');
            
            % Initialize quadrotor model            
            obj@DynamicUnicycle(id, data.Ts, data.d, data.m, data.Iz, initialPos);
            
			% Build controller
            A = @(rho) data.Ak0 + rho * data.Ak1;
            B = @(rho) data.Bk0 + rho * data.Bk1;
            C = @(rho) data.Ck0 + rho * data.Ck1;
            D = @(rho) data.Dk0 + rho * data.Dk1;
            obj.controller = DiscreteLpvDynamics([], A, B, C, D, zeros(8,1));
        end

        function step(obj,ref)
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
            e = ref - obj.position;
            
            % Evaluate controller equation
            u = obj.controller.step(rot * e, rho);
            obj.move(u);       
        end
    end
end
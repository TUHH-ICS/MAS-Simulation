% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef LinearisedQuadrocopterAgent < LinearisedQuadrocopter
    %LINEARISEDQUADROCOPTERAGENT Agent.    
    
    properties(GetAccess = private, SetAccess = immutable)
        controller % Discrete-time LTI controller
    end
   
    methods
        function obj = LinearisedQuadrocopterAgent(id,initialPos)
            %LINEARISEDQUADROCOPTERAGENT Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent to the given initial position.
            
            data = load('quadrotor_model');
            
            % Initialize quadrotor model
            initialVel = zeros(size(initialPos));
            obj@LinearisedQuadrocopter(id, data.Ts, data.m, initialPos, initialVel);
            
            % Assemble controller
            [Ad, Bd, Cd, Dd] = ssdata(data.K);
            obj.controller = DiscreteLtiDynamics(Ad, Bd, Cd, Dd); 
        end

        function step(obj,ref)            
            % Since the equilibirum state is not the origin, compute the
            % error in the state separately and provide the controller with
            % output and state error.
            
            ref_state  = [ref(1);0;ref(2);0;ref(3);0;zeros(6,1)];
            
            % Evaluate agent dynamics
            e_hat = [ref - obj.position; ref_state-obj.state];
            u     = obj.controller.step(e_hat);
            obj.move(u);
        end
    end
end
% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Adwait Datar <adwait.datar@tuhh.de>
%                   Christian Hespe <christian.hespe@tuhh.de>

classdef FlockingAgent2 < DoubleIntegratorAgent
    %FLOCKINGAGENT2 Examplary implementation of double integrator agents
    %that perform a flocking manoeuvre augmented with a source seeking
    % forcing term.
    
    % Define constants of the flocking protocol
    properties(Constant)        
        c_damp          = 3;   % Damping between agents / alignment rule
        c_self_damp     = 1;   % Damping between agents and environment (e.g Viscocity)
        c_gradient      = 1;   % constant multiplying the gradient force
        c_hessian       = 1;   % constant multiplying the hessian damping
    end
    
    properties(GetAccess = private, SetAccess = private)
        QuadModelEst  % Local identified quadratic model
   end
    
    properties(GetAccess = private, SetAccess = immutable)
        interac_field % Interaction field with other agents
        conc_field    % External concentration field
        field_sensor  % sensor that measures the external conc. field        
    end
    
    methods     
        function obj = FlockingAgent2(id, dT, initialPos, initialVel, conc_field,interac_field,field_sensor)
            %FLOCKINGAGENT Construct an instance of this class
            %   Initializes the state space to the given initial position
            %   and velocity.
            
            obj@DoubleIntegratorAgent(id, dT, initialPos, initialVel);
			obj.conc_field    = conc_field;
            obj.interac_field = interac_field;
            obj.field_sensor  = field_sensor;
            
            Data=obj.field_sensor.get_measurement(initialPos);
            obj.QuadModelEst  = obj.field_sensor.quadratic_regression(Data);
        end        
        
        function [grad,hess] = get_gradient_hessian_est(obj)
            %get_gradient_hessian_est
            % This function estimates the gradient and the hessian of the 
            % field at the current agent location.
            
            field_estimate    = obj.position'*obj.QuadModelEst.Q_id*obj.position+obj.QuadModelEst.b_id'*obj.position+obj.QuadModelEst.c_id;
            field_measurement = obj.conc_field.get_field_value_at(obj.position)+obj.field_sensor.noise_bound*(-1+2*rand);
            
            if abs(field_estimate-field_measurement) >= obj.field_sensor.noise_bound
                Data = obj.field_sensor.get_measurement(obj.position);
                obj.QuadModelEst=obj.field_sensor.quadratic_regression(Data);
            end
            
            grad = 2*obj.QuadModelEst.Q_id*obj.position+obj.QuadModelEst.b_id;
            hess = 2*obj.QuadModelEst.Q_id;
            
            % Can get the true gradient and hessians at obj.position 
            % for a posterior analysis
            %grad=obj.conc_field.get_true_gradient(obj.position);
        end
        
        function step(obj)
            % Receive messages from the network
            messages = obj.receive();
            
            % Implement the flocking protocol
            u = zeros(2, 1);
            for message = messages                                
                % Force due to interaction field potential field and the
                % adjacency element betwen agents
                [force,a] = obj.interac_field.get_interaction_force(message.data.position - obj.position);
                u = u + force;                     
                % Calculate alignment force
                u = u + FlockingAgent2.c_damp * a * (message.data.velocity - obj.velocity);
            end
            
            % Calculate force on the agent from the source field
            [grad,hess] = obj.get_gradient_hessian_est();
            u = u - obj.c_hessian*hess*obj.velocity - obj.c_gradient*grad;
            
            % Self_damping
            u = u - FlockingAgent2.c_self_damp*obj.velocity; 
                        
            % Evaluate double integrator dynamics
            obj.move(u);
            
            % Send message to network, include position and velocity
            data = struct;
            data.position = obj.position;
            data.velocity = obj.velocity;
            obj.send(data)
        end
    end
end


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
            
            % Intialize a zero model of the field
            obj.QuadModelEst.Q_id  = zeros(obj.dim);
            obj.QuadModelEst.b_id  = zeros(obj.dim,1);
            obj.QuadModelEst.c_id  = 0;
        end        
        
        function [grad,hess] = get_gradient_hessian_est(obj,field_data)
            %get_gradient_hessian_est
            % This function estimates the gradient and the hessian of the 
            % field at the current agent location.
            
            field_estimate  = obj.position'*obj.QuadModelEst.Q_id*obj.position+obj.QuadModelEst.b_id'*obj.position+obj.QuadModelEst.c_id;
            field_data_self = field_data.values(1);
            
            % We use the following heuristic for identifying the underlying
            % field which has been observed to work for an inverted gaussian field:
            % 1. If we have less than three data points, we assume a zero
            % model since the identfied model wouldn\t be reliable
            % 2. If we have three to ten data points, we try to identify a
            % linear model which has three model parameters (grad and bias)
            % 3. If we have more than ten data points, we identify a
            % quadratic model which has 6 model parameters (hess, grad and bias)
            data_size=size(field_data.values,2);
            if abs(field_estimate-field_data_self) >= obj.field_sensor.noise_bound
                if (data_size>=3) && (data_size<10)
                    obj.QuadModelEst=obj.field_sensor.linear_regression(field_data);
                elseif (data_size>=10)
                    obj.QuadModelEst=obj.field_sensor.quadratic_regression(field_data);
                else
                    obj.QuadModelEst.Q_id  = zeros(obj.dim);
                    obj.QuadModelEst.b_id  = zeros(obj.dim,1);
                    obj.QuadModelEst.c_id  = 0;
                end
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
            
            % Collect local field data
            field_data_self=obj.field_sensor.get_measurement(obj.position);
            % Add local data first and then add the data from neighbors in
            % the for loop
            field_data_all=field_data_self;
            
            % Implement the flocking protocol
            u = zeros(2, 1);
            for message = messages
                data=message.data;
                % Force due to interaction field potential field and the
                % adjacency element betwen agents
                [force,a] = obj.interac_field.get_interaction_force(data.position - obj.position);
                u = u + force;                     
                % Calculate alignment force
                u = u + FlockingAgent2.c_damp * a * (data.velocity - obj.velocity);
                
                % Disregard received data from agents outside the sensing radius (a=0)
                if a>0
                    field_data_all.positions=[field_data_all.positions,data.field_data.positions];
                    field_data_all.values=[field_data_all.values,data.field_data.values];
                end
                
            end
            
            % Calculate force on the agent from the source field
            [grad,hess] = obj.get_gradient_hessian_est(field_data_all);
            u = u - obj.c_hessian*hess*obj.velocity - obj.c_gradient*grad;
            
            % Self_damping
            u = u - FlockingAgent2.c_self_damp*obj.velocity; 
                        
            % Evaluate double integrator dynamics
            obj.move(u);
            
            % Send message to network, include position and velocity
            data = struct;
            data.position = obj.position;
            data.velocity = obj.velocity;            
            data.field_data= field_data_self;
            obj.send(data)
        end
    end
end


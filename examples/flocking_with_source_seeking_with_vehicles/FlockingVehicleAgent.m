% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Adwait Datar <adwait.datar@tuhh.de>
%                   Christian Hespe <christian.hespe@tuhh.de>

classdef FlockingVehicleAgent < DynamicAgent
    %FLOCKINGVEHICLEAGENT Examplary implementation of double integrator virtual particle with vehicle models tracking these virtual particles
    % to perform a flocking manoeuvre augmented with a source seeking
    % forcing term.
    
    % Define constants of the flocking protocol
    properties(Constant)        
        c_damp          = 3;   % Damping between agents / alignment rule        
        c_gradient      = 0.1;   % constant multiplying the gradient force
        c_hessian       = 0;   % constant multiplying the hessian damping
        c_interact      = 1e-2; % constant multiplying the interaction force
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        dim          % dimension of the virtual particle
        Vehicle      % Model of the vehicle 
        arch         % Architecture: 1-> coupled(transmit vehicle pos and vel) OR 2 -> decoupled(transmit virtual pos and vel)
        no_veh_steps % number of vehicle time-steps for every FlockingVehicleAgent time-step
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the agent
        position_vir % Current position of the virtual particle
        velocity % Current velocity of the agent
        velocity_vir % Current velocity of the virtual particle 
        Vehicle_state % Current state vector of the vehicle												   
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
        function obj = FlockingVehicleAgent(id, dT, initialPos, initialVel, conc_field,interac_field,field_sensor,Vehicle,arch,mass,c_fric)
            %FLOCKINGAGENT Construct an instance of this class
            %   Initializes the state space to the given initial position
            %   and velocity.

            dim=length(initialPos); 
            % Define the dynamics of the virtual second order particle
            A = kron(eye(dim), [0 1; 0 -c_fric/mass]);
            B = kron(eye(dim), [0; 1/mass]);
            [Ad, Bd] = ltiDiscretization(A, B, dT, 'zoh');              
            % Construct discrete-time state space model
            x0 = kron(initialPos, [1; 0]) + kron(initialVel, [0; 1]);
            dynamics = DiscreteLtiDynamics(Ad, Bd, [], [], x0);
            obj@DynamicAgent(id, dT, dynamics);            
            
            
            obj.dim = dim;
            if dim ~= length(initialVel)
                error('Position and velocity vectors must have the same dimensions!')
            end
            
            obj.conc_field    = conc_field;
            obj.interac_field = interac_field;
            obj.field_sensor  = field_sensor;
            
            % Intialize a zero model of the field
            obj.QuadModelEst.Q_id  = zeros(obj.dim);
            obj.QuadModelEst.b_id  = zeros(obj.dim,1);
            obj.QuadModelEst.c_id  = 0;
            
            % Intialize the vehicle model
            obj.Vehicle  = Vehicle;
            obj.arch=arch;
            obj.no_veh_steps=obj.dT/obj.Vehicle.dT;
            if obj.no_veh_steps~=floor(obj.no_veh_steps)
                error('dT must be an integer multiple of %f',obj.Vehicle.dT)
            end
        end
        
        function value = get.position(obj)
            %GET.POSITION Implementation of the dependent position
            %property.
            %   This function returns the position of the Vehicle object
            %   associated with the agent            
            value = obj.Vehicle.position(1:2); % Current implementation supports dim=2 and so only returning first two dims
        end        
        
        function value = get.velocity(obj)
            %GET.VELOCITY Implementation of the dependent velocity
            %property.
            %   This function returns the velocity of the Vehicle object
            %   associated with the flocking agent            
            value = obj.Vehicle.velocity(1:2); % Current implementation supports dim=2 and so only returning first two dims
        end
        function value = get.position_vir(obj)
            %GET.POSITION Implementation of the dependent position
            %property.
            %   This function is simply a projection from the state space 
            %   of the virtual particle into the position space.
            
            value = obj.state(2*(1:obj.dim) - 1);
        end
        
        function value = get.velocity_vir(obj)
            %GET.VELOCITY Implementation of the dependent velocity
            %property.
            %   This function is simply a projection from the state space
            %   of the virtual particle into the velocity space.
            
            value = obj.state(2*(1:obj.dim));
        end
		function value = get.Vehicle_state(obj)
            %GET.VEHICLE_STATE Implementation of the dependent
            %VEHICLE_STATE property
            %   This function returns the state vector of the Vehicle 
            % object associated with the agent            
            value = obj.Vehicle.state;
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
            
            % Set self position and velocity for computing the error based
            % on the set architecture
            [self_pos,self_vel]=get_self_ref(obj);
            
            
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
                [force,a] = obj.interac_field.get_interaction_force(data.position - self_pos);
                u = u + obj.c_interact*force;                     
                % Calculate alignment force
                u = u + obj.c_damp * a * (data.velocity - self_vel);
                
                % Disregard received data from agents outside the sensing radius (a=0)
                if a>0
                    field_data_all.positions=[field_data_all.positions,data.field_data.positions];
                    field_data_all.values=[field_data_all.values,data.field_data.values];
                end
            end
            
            % Calculate force on the agent from the source field
            [grad,hess] = obj.get_gradient_hessian_est(field_data_all);
            u = u - obj.c_hessian*hess*self_vel - obj.c_gradient*grad;           

                        
            % Move the virtual second order particle which is then set as
            % reference to the vehicle 
            obj.move(u);            
                                  
            % Set the position of the virtual particle as reference to
            % Vehicle
            switch(length(obj.Vehicle.position))
                case 1
                    error('Dimensions of virtual particle and vehicle do not match')
                case 2
                    veh_ref=[obj.position_vir];
                case 3
                    veh_ref=[obj.position_vir;0];
            end
			for i=1:obj.no_veh_steps
				obj.Vehicle.step(veh_ref);
			end            
            
            % Send message to network, include position and velocity
            data = struct; 
            [data.position,data.velocity]=get_self_ref(obj);
            data.field_data= field_data_self;
            obj.send(data)
        end
    end
	methods(Access=private)
		function [pos,vel]=get_self_ref(obj)
            %GET_SELF_REF
            %   This function returns the agent position based on the archtiecture
            % (Used later for computing forces and for transmitting to
            % other agents)
            switch obj.arch
                case 1 % Coupled(transmit vehicle pos and vel)                    
                    pos = obj.position;
                    vel = obj.velocity;
                case 2 % Decoupled(transmit virtual pos and vel) architecture
                    pos = obj.position_vir;
                    vel = obj.velocity_vir;
            end
        end
	end
end


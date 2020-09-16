% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef HippoCampusCL < DynamicAgent
    %HIPPOCAMPUSCL Agent implementation with a dynamic unicycle model.
    %   This model uses a pre-defined closed-loop LPV system that contains
    %   the dynamics of the controller and the hippocampus
    
    properties(GetAccess = public, SetAccess = private)
        ref % Reference of the HippoCampus
    end
    
    properties(GetAccess = private, SetAccess = private)
        t   % Current time of the simulation
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the unicycle
        velocity % Current velocity of the unicycle
    end
    
    methods
        function obj = HippoCampusCL(id, dT, initialPos)
            %HIPPOCMAPUSAGENT Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent and consensus protocol to the given initial position.
            
            % Load data from synthesis script
            data = load('HippoCampusCL');
            
            % Build Look-Up-Tables for system dynamics
            A = @(rho) mdlerp(data.A, data.Dom, rho);
            B = @(rho) mdlerp(data.B, data.Dom, rho);
            C = @(rho) mdlerp(data.C, data.Dom, rho);
            D = @(rho) mdlerp(data.D, data.Dom, rho);
            
            % Assemble dynamic system
            x0 = [zeros(24, 1); initialPos; zeros(9, 1)];
            rho = @(t, x, w) x([34:36, 28:30]);
            dynamics = ContinuousLpvDynamics(dT, rho, A, B, C, D, x0);
            
            % Create object with given parameters
            obj@DynamicAgent(id, dT, dynamics);
            
            % Initialize internal state of the simulation model
            obj.ref   = [0; 3; 0];
            obj.t     = 0;
        end

        function value = get.position(obj)
            %GET.POSITION Implementation of the dependent position
            %property.
            %   This function is simply a projection from the state space 
            %   of the agents into the position space.
            
            value = obj.state(25:27);
        end
        
        function value = get.velocity(obj)
            %GET.VELOCITY Implementation of the dependent velocity
            %property.
            %   This function is simply a projection from the state space 
            %   of the agents into the position space.
            
            value = obj.state(28:30);
        end
        
        function step(obj)
            % The HippoCampus should follow a figure 8 trajectory
            obj.ref = [ 3*sin(2*pi*obj.t/200) ;
                        3*cos(2*pi*obj.t/200) ;
                        obj.t/100             ];
                        
            % Apply control inputs to the agent
            obj.move(obj.ref);
            obj.t = obj.t + obj.dT;
        end
    end
end
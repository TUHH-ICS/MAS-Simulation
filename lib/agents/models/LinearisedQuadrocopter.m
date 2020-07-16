% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef LinearisedQuadrocopter < DynamicAgent
    %LINEARISEDQUADROCOPTER Agent class that implements a linearied model
    %of a quadrocopter.
    %   The agent dynamics are approximated as a LTI system using Jacobian
    %   linearization. 
    %
    %   See D. Lara, A. Sanchez, R. Lozano, P. Castillo.  Real-time 
    %   embedded control system for VTOL aircrafts: Application to
    %   stabilize a quad-rotor helicopter, 2006
    
    properties(Constant)
        g = 9.81 % Gravitational force constant
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        m        % Mass of the quadrocopter
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the agent
        velocity % Current velocity of the agent
    end
    
    methods
        function obj = LinearisedQuadrocopter(id, dT, m, initialPos)
            %LINEARISEDQUADROCOPTER Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent with the correct initial position.
                        
            % Define continuous-time state-space matrices
            A = [ 0 1 0 0 0 0 0 0  0 0 0 0 ;  % x
                  0 0 0 0 0 0 0 0 -g 0 0 0 ;  % xdot
                  0 0 0 1 0 0 0 0  0 0 0 0 ;  % y
                  0 0 0 0 0 0 0 0  0 0 g 0 ;  % ydot
                  0 0 0 0 0 1 0 0  0 0 0 0 ;  % z
                  0 0 0 0 0 0 0 0  0 0 0 0 ;  % zdot
                  0 0 0 0 0 0 0 1  0 0 0 0 ;  % psi
                  0 0 0 0 0 0 0 0  0 0 0 0 ;  % psidot
                  0 0 0 0 0 0 0 0  0 1 0 0 ;  % theta
                  0 0 0 0 0 0 0 0  0 0 0 0 ;  % thetadot
                  0 0 0 0 0 0 0 0  0 0 0 1 ;  % phi
                  0 0 0 0 0 0 0 0  0 0 0 0 ]; % phidot
            B = [ 0   0 0 0 ;
                  0   0 0 0 ;
                  0   0 0 0 ;
                  0   0 1 0 ;
                  0   0 0 0 ;
                  1/m 0 0 0 ;
                  0   0 0 0 ;
                  0   1 0 0 ;
                  0   0 0 0 ;
                  0   0 1 0 ;
                  0   0 0 0 ;
                  0   0 0 1 ];
            [Ad, Bd] = ltiDiscretization(A, B, dT);
              
            % Construct discrete-time state space model
            x0 = [ kron(initialPos, [1; 0]); zeros(6,1) ];
            dynamics = DiscreteLtiDynamics(Ad, Bd, [], [], x0);
            
            obj@DynamicAgent(id, dT, dynamics);
            obj.m = m;
        end
        
        function value = get.position(obj)
            %GET.POSITION Implementation of the dependent position
            %property.
            %   This function is simply a projection from the state space 
            %   of the agents into the position space.
            
            value = [obj.state(1); obj.state(3); obj.state(5)];
        end
        
        function value = get.velocity(obj)
            %GET.VELOCITY Implementation of the dependent velocity
            %property.
            %   This function is simply a projection from the state space
            %   of the agents into the velocity space.
            
            value = [obj.state(2); obj.state(4); obj.state(6)];
        end
    end
end


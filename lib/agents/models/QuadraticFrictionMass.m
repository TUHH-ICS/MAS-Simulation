%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

classdef QuadraticFrictionMass < DynamicAgent
    %QUADRATICFRICTIONMASS Class that implements the dynamics of a moving
    %mass with quadratic friction.
    %   This dynamics is a useful model for satellites or underwater
    %   vehicles moving with high velocity, where turbulence occours and
    %   thus the friction depends on the squared velocity.
    %
    %   The corresponding ODE is
    %   m * ydotdot + b * ||ydot|| * ydot = u
    
    properties(GetAccess = public, SetAccess = immutable)
        dim      % Number of independent integrators
        m        % Mass of the model
        b        % Friction coefficient
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the agent
        velocity % Current velocity of the agent
    end
    
    methods
        function obj = QuadraticFrictionMass(id, dT, m, b, initialPos, initialVel)
            %QUADRATICFRICTIONMASS Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent with the correct initial position and velocity.
            %
            %   id          Id of the agent in the network
            %   dT          Desired sampling time
            %   m           Mass of the model
            %   b           Coefficient of friction
            %   initialPos  Initial position of the agent
            %   initialVel  Initial velocity of the agent
            
            dim = length(initialPos);
            if dim ~= length(initialVel)
                error('Position and velocity vectors must have the same dimensions!')
            end
              
            % Construct discrete-time state space model using Euler
            % discretization
            x0 = kron(initialPos, [1; 0]) + kron(initialVel, [0; 1]);
            f  = @(t, x, u) QuadraticFrictionMass.odeFun(t, x, u, m, b);
            fd = nonlinearDiscretization(f, dT, 'euler');
            dynamics = DiscreteNonlinearDynamics(fd, [], x0);
            
            obj@DynamicAgent(id, dT, dynamics);
            obj.dim = dim;
            obj.b   = b;
            obj.m   = m;
        end
        
        function value = get.position(obj)
            %GET.POSITION Implementation of the dependent position
            %property.
            %   This function is simply a projection from the state space 
            %   of the agents into the position space.
            
            value = obj.state(2*(1:obj.dim) - 1);
        end
        
        function value = get.velocity(obj)
            %GET.VELOCITY Implementation of the dependent velocity
            %property.
            %   This function is simply a projection from the state space
            %   of the agents into the velocity space.
            
            value = obj.state(2*(1:obj.dim));
        end
    end
    
    methods(Static, Sealed, Access = private)
        function xdot = odeFun(~, x, u, m, b)
            %ODEFUN ODE representation of a moving mass with quadratic
            %friction.

            dim  = length(x)/2;
            xdot = zeros(2*dim, 1);
            vel  = norm(x(2*(1:dim)));
            
            for j = 1:dim
                i = 2*j;
                xdot(i-1) = x(i);
                xdot(i)   = 1/m * (u(j) - b*vel*x(i));
            end
        end
    end
end

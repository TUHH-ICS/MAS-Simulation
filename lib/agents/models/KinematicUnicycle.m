%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe, Philipp Hastedt
%---------------------------------------------------------------------------------------------------

classdef KinematicUnicycle < DynamicAgent
    %KINEMATICNICYCLE Class that implements the nonlinear dynamics of a
    %kinematic unicycle.
    %   state vector: x = [xi; yi; psi]
    %       xi,yi   :   position in inertial coordinate frame
    %       psi     :   rotation about z-axis
    %   inputs: u = [F; tau]
    %       ub       :   velocity in x-direction in body coordinate frame
    %       dpsi     :   angular velocity
    %   dynamics:
    %       xdot(1) = u(1) * cos(x(3))
    %       xdot(2) = u(1) * sin(x(3))
    %       xdot(3) = u(2)
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position        % Current position of the unicycle
        velocity        % Current velocity of the unicycle in inertial coordinates
        velocityBody    % Current velocity of the unicycle in body coordinates
        attitude        % Current attitude of the unicycle
    end
    
    methods
        function obj = KinematicUnicycle(id, dT, initialPos, initialAtt)
            %KINEMATICUNICYCLE Construct an instance of this class
            %   Sets up the correct unicycle dynamics and initializes the
            %   unicycle with the given position.
            %
            %   id          Id of the agent in the network
            %   dT          Desired sampling time
            %   initialPos  Initial position of the agent [xi; yi]
            %   initialAtt  Initial attitude of the agent [psi]
                        
            % Initialize discrete-time dynamics. The discrete-time model is
            % calculated by a Euler discretization.
            x0 = [ initialPos; initialAtt];
            f  = @(t, x, u) KinematicUnicycle.odeFun(t, x, u);
            fd = nonlinearDiscretization(f, dT);
            dynamics = DiscreteNonlinearDynamics(fd, [], x0);
            
            % Create object with given parameters
            obj@DynamicAgent(id, dT, dynamics);
        end
        
        function [posHandle, velHandle] = getStateWithHandle(obj, handleLength)
            %GETSTATEWITHHANDLE Calculate state for unicycle with handle in
            %inertial coordinates
            psi = obj.attitude;
            posHandle = obj.position + handleLength * [cos(psi); sin(psi)];
            velHandle = [obj.u(1) * cos(psi) - obj.u(2) * handleLength * sin(psi)
                         obj.u(1) * sin(psi) + obj.u(2) * handleLength * cos(psi)];
        end
        
        function value = get.position(obj)
            %GET.POSITION Implementation of the dependent position
            %property.
            %   This function is simply a projection from the state space 
            %   of the agents into the position space.
            
            value = obj.state(1:2);
        end
        
        function value = get.velocity(obj)
            %GET.VELOCITY Implementation of the dependent velocity
            %property.
            %   This property returns the velocity vector in inertial
            %   coordinates   
            psi   = obj.attitude;
            value = obj.u(1) * [ cos(psi); sin(psi) ];
        end
        
        function value = get.velocityBody(obj)
            %GET.VELOCITYBODY Implementation of the dependent velocityBody
            %property.
            %   This property returns the velocity vector in body
            %   coordinates
            value = [obj.u(1); 0];
        end
        
        function value = get.attitude(obj)
            %GET.ATTITUDE Implementation of the dependent attitude 
            %property.
            %   This property returns the attitude
            value = obj.state(3);
        end
    end
    
    methods(Static, Sealed, Access = private)
        function xdot = odeFun(~, x, u)
            %ODEFUN ODE representation of a kinematic unicycle with handle

            xdot    = zeros(size(x));
            xdot(1) = u(1) * cos(x(3)); % xi
            xdot(2) = u(1) * sin(x(3)); % yi
            xdot(3) = u(2);             % psi
        end
    end
end
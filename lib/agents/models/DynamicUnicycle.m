%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe, Philipp Hastedt
%---------------------------------------------------------------------------------------------------

classdef DynamicUnicycle < DynamicAgent
    %DYNAMICUNICYCLE Class that implements the nonlinear dynamics of a
    %dynamic unicycle.
    %   state vector: x = [xi; yi; psi; ub; dpsi]
    %       xi,yi   :   position in inertial coordinate frame
    %       psi     :   rotation about z-axis
    %       ub      :   velocity in x-direction in body coordinate frame
    %       dpsi    :   angular velocity
    %   inputs: u = [F; tau]
    %       F       :   force in x-direction in body coordinate frame
    %       Tau     :   torque about z-axis
    %   model parameters:
    %       M       :   mass
    %       Iz      :   inertia about z-axis
    %   dynamics:
    %       xdot(1) = x(4) * cos(x(3))
    %       xdot(2) = x(4) * sin(x(3))
    %       xdot(3) = x(5)
    %       xdot(4) = u(1) / M
    %       xdot(5) = u(2) / Iz
    
    properties(GetAccess = public, SetAccess = immutable)
        M   % Mass of the unicycle
        Iz  % Moment of inertia around the z axis
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position        % Current position of the unicycle
        velocity        % Current velocity of the unicycle in inertial coordinates
        velocityBody    % Current velocity of the unicycle in body coordinates
        attitude        % Current attitude of the unicycle
        angularVelocity % Current angular velocity of the unicycle
    end
    
    methods
        function obj = DynamicUnicycle(id, dT, M, Iz, initialPos, initialAtt, initialVel, initialAngVel, varargin)
            %DYNAMICUNICYCLE Construct an instance of this class
            %   Sets up the correct unicycle dynamics and initializes the
            %   unicycle with the given position.
            %
            %   id              Id of the agent in the network
            %   dT              Desired sampling time
            %   M               Mass of the unicycle
            %   Iz              Rotational moment of inertia around z axis
            %   initialPos      Initial position of the agent [xi; yi]
            %   initialAtt      Initial attitude of the agent [psi]
            %   initialVel      (optional) Initial velocity of the agent [ub]
            %   initalAngVel    (optional) Initial angular velocity of the agent [dpsi] 
                        
            % The state can be initialized with either position and
            % attitude or complete initial state.
            % state vector: x = [xi; yi; psi; ub; dpsi]
            if nargin == 6
                x0 = [initialPos; initialAtt; 0; 0];
            else
                x0 = [initialPos; initialAtt; initialVel; initialAngVel];
            end
            f  = @(t, x, u) DynamicUnicycle.odeFun(t, x, u, M, Iz);
            fd = nonlinearDiscretization(f, dT);
            dynamics = DiscreteNonlinearDynamics(fd, [], x0);
            
            % Create object with given parameters
            obj@DynamicAgent(id, dT, dynamics);
            obj.M  = M;
            obj.Iz = Iz;
        end

        function [posHandle, velHandle] = getStateWithHandle(obj, handleLength)
            %GETSTATEWITHHANDLE Calculate state for unicycle with handle in
            %inertial coordinates
            psi = obj.attitude;
            posHandle = obj.position + handleLength * [cos(psi); sin(psi)];
            velHandle = [obj.velocityBody(1) * cos(psi) - obj.angularVelocity * handleLength * sin(psi);
                         obj.velocityBody(1) * sin(psi) + obj.angularVelocity * handleLength * cos(psi)];
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
            psi = obj.attitude;
            value = obj.state(4)*[cos(psi); sin(psi)];
        end
        
        function value = get.velocityBody(obj)
            %GET.VELOCITYBODY Implementation of the dependent velocityBody
            %property.
            %   This property returns the velocity vector in body
            %   coordinates
            value = [obj.state(4); 0];
        end
        
        function value = get.attitude(obj)
            %GET.ATTITUDE Implementation of the dependent attitude 
            %property.
            %   This property returns the attitude
            value = obj.state(3);
        end 
        
        function value = get.angularVelocity(obj)
            %GET.ANGULARVELOCITY Implementation of the dependent
            %angularVelocity property.
            %   This property returns the angular velocity
            value = obj.state(5);
        end 
    end
    
    methods(Static, Sealed, Access = private)
        function xdot = odeFun(~, x, u, M, Iz)
            %ODEFUN ODE representation of a dynamic unicycle
            xdot    = zeros(size(x));
            xdot(1) = x(4) * cos(x(3));
            xdot(2) = x(4) * sin(x(3));
            xdot(3) = x(5);
            xdot(4) = u(1) / M;
            xdot(5) = u(2) / Iz;
        end
    end
end

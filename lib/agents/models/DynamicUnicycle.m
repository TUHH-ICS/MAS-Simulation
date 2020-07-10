classdef DynamicUnicycle < DynamicAgent
    %DYNAMICUNICYCLE Class that implements the nonlinear dynamics of a
    %dynamic unicycle with handle.
    %   The continuous-time dynamics are discretized using a Euler
    %   disretization.
    
    properties(GetAccess = public, SetAccess = immutable)
        m   % Mass of the unicycle
        Iz  % Moment of inertia around the z axis
        d   % Length of the handle
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the unicycle
        velocity % Current velocity of the unicycle
    end
    
    methods
        function obj = DynamicUnicycle(id, dT, d, m, Iz, initialPos)
            %DYNAMICUNICYCLE Construct an instance of this class
            %   Sets up the correct unicycle dynamics and initializes the
            %   unicycle with the given position.
                        
            % Initialize discrete-time dynamics. The discrete-time model is
            % calculated by a Euler discretization.
            x0 = [ initialPos; zeros(3,1) ];
            f = @(k, x, u) x + dT * DynamicUnicycle.odeFun(dT * k, x, u, d, m, Iz);
            dynamics = DiscreteNonlinearDynamics(f, [], x0);
            
            % Create object with given parameters
            obj@DynamicAgent(id, dT, dynamics);
            obj.m  = m;
            obj.Iz = Iz;
            obj.d  = d;
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
            %   This property should return the velocity vector, and not
            %   just the magnitude of the velocity. Therefore, we construct
            %   a unit vector in the current direction of the unicycle and
            %   multiply it with the speed.
            
            phi   = obj.state(4);
            value = obj.state(3) * [ cos(phi); sin(phi) ];
        end 
    end
    
    methods(Static, Sealed, Access = private)
        function xdot = odeFun(~, x, u, d, m, Iz)
            %ODEFUN ODE representation of a dynamic unicycle with handle

            xdot    = zeros(size(x));
            xdot(1) = x(3) * cos(x(4)) - x(5) * d * sin(x(4)); % qx
            xdot(2) = x(3) * sin(x(4)) + x(5) * d * cos(x(4)); % qy
            xdot(3) = u(1) / m;                                % v
            xdot(4) = x(5);                                    % phi
            xdot(5) = u(2) / Iz;                               % omega
        end
    end
end
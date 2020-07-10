classdef KinematicUnicycle < DynamicAgent
    %KINEMATICNICYCLE Class that implements the nonlinear dynamics of a
    %kinematic unicycle with handle.
    %   The continuous-time dynamics are discretized using a Euler
    %   disretization.
    
    properties(GetAccess = public, SetAccess = immutable)
        d   % Length of the handle
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the unicycle
        velocity % Current velocity of the unicycle
    end
    
    methods
        function obj = KinematicUnicycle(id, dT, d, initialPos)
            %KINEMATICUNICYCLE Construct an instance of this class
            %   Sets up the correct unicycle dynamics and initializes the
            %   unicycle with the given position.
                        
            % Initialize discrete-time dynamics. The discrete-time model is
            % calculated by a Euler discretization.
            x0 = [ initialPos; 0];
            f = @(k, x, u) x + dT * KinematicUnicycle.odeFun(dT * k, x, u, d);
            dynamics = DiscreteNonlinearDynamics(f, [], x0);
            
            obj@DynamicAgent(id, dT, dynamics);
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
            
            phi   = obj.state(3);
            value = obj.u(1) * [ cos(phi); sin(phi) ];
        end 
    end
    
    methods(Static, Sealed, Access = private)
        function xdot = odeFun(~, x, u, d)
            %ODEFUN ODE representation of a kinematic unicycle with handle

            xdot    = zeros(size(x));
            xdot(1) = u(1) * cos(x(3)) - u(2) * d * sin(x(3)); % qx
            xdot(2) = u(1) * sin(x(3)) + u(2) * d * cos(x(3)); % qy
            xdot(3) = u(2);                                    % phi
        end
    end
end
classdef FormationUnicycle < BaseAgent
    
    properties(Constant)
        epsilon = 0.0005; % Convergence speed of the consensus protocol
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        m   % Mass of the unicycle
        Iz  % Moment of inertia
        d   % Length of the handle
        
        ref % Formation reference
    end
    
    properties(GetAccess = public, SetAccess = protected)
        consens  % State of the consensus protocol
    end
    
    properties(GetAccess = private, SetAccess = immutable)
        dynamics % Discrete-time LTI agent dynamics
        controller
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the agent
        velocity % Current velocity of the agent
        state    % Dynamic state of the agent
    end
    
    methods
        function obj = FormationUnicycle(id, initialPos, reference)
            %FORMATIONUNICYCLE Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent and consensus protocol to the given initial position.
            
            % Load data from synthesis script
            data = load('unicycle_controller');
            
            obj@BaseAgent(id, data.Ts);
            obj.m  = data.m;
            obj.Iz = data.Iz;
            obj.d  = data.d;
            
            x0 = [ initialPos; zeros(3,1) ];
            f = @(k, x, u) x + obj.dT * unicycleFun(obj.dT * k, x, u);
            obj.dynamics = DiscreteNonlinearDynamics(f, [], x0);
            
            % Build controller
            A = @(rho) data.Ak0 + rho * data.Ak1;
            B = @(rho) data.Bk0 + rho * data.Bk1;
            C = @(rho) data.Ck0 + rho * data.Ck1;
            D = @(rho) data.Dk0 + rho * data.Dk1;
            obj.controller = DiscreteLpvDynamics([], A, B, C, D, zeros(8,1));
            
            % Initialize the consensus protocol to the initial position of
            % the agent itself
            obj.consens = initialPos;
            
            % Set the formation reference if one is given to the agent
            if nargin <= 2
                obj.ref = zeros(2,1);
            else
                obj.ref = reference;
            end
        end

        function value = get.state(obj)
            %GET.STATE Implementation of the dependent state property.
            value = obj.dynamics.x;
        end
        
        function value = get.position(obj)
            %GET.POSITION Implementation of the dependent position
            %property.
            %   This function is simply a projection from the state space 
            %   of the agents into the position space.
            value = obj.dynamics.x(1:2);
        end
        
        function value = get.velocity(obj)
            %GET.VELOCITY Implementation of the dependent velocity
            %property.
            %   This function is simply a projection from the state space
            %   of the agents into the velocity space.
            dir = @(phi) [ cos(phi); sin(phi) ];
            value = obj.dynamics.x(3) * dir(obj.dynamics.x(4));
        end 

        function step(obj)
            % Receive messages from the network
            messages = obj.receive();
            
            if ~isempty(messages)
                % Calculate new formation reference. We use the normalized
                % Laplacian, therefore we calculate the mean of the
                % positions, not the sum
                data        = [messages.data];
                positions   = [data.position];
                dist        = obj.consens - obj.ref - mean(positions, 2);
                obj.consens = obj.consens - obj.epsilon * dist;
            end
            
            % Extract state information from the unicycle model
            phi   = obj.dynamics.x(4);
            omega = obj.dynamics.x(5);

            % For the controller synthesis, we used a rotated frame of
            % reference for the agents. As these are state feedback
            % controllers, we need to also rotate the frame of reference
            % here to be consistent with the synthesis.
            rot = [ cos(phi), sin(phi); -sin(phi), cos(phi) ];

            % Calculate current scheduling parameter vt
            rho = obj.d * omega;
            
            % Calculate local tracking error of the unicycle
            e = obj.consens - obj.position;
            
            % Evaluate controller equation
            u = obj.controller.step(rot * e, rho);
            obj.dynamics.step(u);
                        
            % Send message to network, include only the position 
            data = struct;
            data.position = obj.consens - obj.ref;
            obj.send(data)
        end
    end
end

function xdot = unicycleFun(~, x, u)
    %UNICYCLEFUN ODE representation of a dynamic unicycle

    m = 1; % Mass of the unicycle
    I = 1; % Moment of inertia
    d = 0.2; % Length of the handle

    xdot    = zeros(size(x));
    xdot(1) = x(3) * cos(x(4)) - x(5) * d * sin(x(4)); % qx
    xdot(2) = x(3) * sin(x(4)) + x(5) * d * cos(x(4)); % qy
    xdot(3) = u(1) / m;                                % v
    xdot(4) = x(5);                                    % phi
    xdot(5) = u(2) / I;                                % omega
end
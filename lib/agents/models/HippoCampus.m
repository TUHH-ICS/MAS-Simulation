% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef HippoCampus < DynamicAgent
    %HIPPOCAMPUS Class that implements the dynamic model of the HippoCampus
    %AUV developed at the MuM institute.
    %   There are several papers that describe this vehicle. Most relatant
    %   are for this implementation:
    %
    %   A. Hackbarth; E. Kreuzer & E. Solowjow. "HippoCampus: A Micro
    %   Underwater Vehicle for Swarm Applications", 2015
    %
    %   D.-A. Duecker; E. Kreuzer; G. Maerker & E. Solowjow. "Parameter
    %   Identification for Micro Underwater Vehicles", 2018
    %
    %   For an implementation of flocking behaviour with this model, see:
    %
    %   A. Attallah; A. Datar & H. Werner. "Flocking of Linear Parameter
    %   Varying Agents: Source Seeking Application with Underwater
    %   Vehicles", 2020
    
    properties(Constant)
        zg    =  0.05    % [m]
        g     =  9.81    % [m/s^2]
        
        m     =  1.43    % [kg]
        Ix    =  0.00241 % [kgm^2]
        Iy    =  0.01072 % [kgm^2]
        Iz    =  0.01072 % [kgm^2]
        Xudot = -1.11    % [kg]
        Yvdot = -2.80    % [kg]
        Zwdot = -2.80    % [kg]
        Kpdot = -0.0018  % [kgm^2]
        Mqdot = -0.0095  % [kgm^2]
        Nrdot = -0.0095  % [kgm^2]
        Xu    = -4.56    % [kg/m]
        Yv    = -17.36   % [kg/m]
        Zw    = -17.36   % [kg/m]
        Kp    = -0.0028  % [kgm^2]
        Mq    = -0.0476  % [kgm^2]
        Nr    = -0.0476  % [kgm^2]
    end
    
    properties(GetAccess = public, SetAccess = private)
        u_sat % Contains the control inputs before saturation
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        saturation % Control whether the control inputs will be saturated
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the unicycle
        velocity % Current velocity of the unicycle
    end
    
    methods
        function obj = HippoCampus(id, dT, initialPos, saturation)
            %HIPPOCAMPUS Construct an instance of this class
            %   Sets up the discrete-time system equations for the
            %   nonlinear HippoCampus model with the correct initial
            %   conditions.
            %
            %   id          Id of the agent in the network
            %   dT          Desired sampling time
            %   initialPos  Initial position of the agent
            %   saturation  Activates or deactivates saturation of the
            %               control input
            
            % Set default initial position
            if nargin <= 2
                initialPos = zeros(3,1);
            end
            
            % Initialize discrete-time dynamics. The discrete-time model is
            % calculated by a Euler discretization.
            x0 = [ initialPos; zeros(9,1) ];
            fd = nonlinearDiscretization(@HippoCampus.odeFun, dT, 'rk4');
            dynamics = DiscreteNonlinearDynamics(fd, [], x0);
            
            % Create object with given parameters
            obj@DynamicAgent(id, dT, dynamics);
            
            % Deactivate saturation by default
            if nargin <= 3
                obj.saturation = false;
            else
                obj.saturation = saturation;
            end
        end
        
        function value = get.position(obj)
            %GET.POSITION Implementation of the dependent position
            %property.
            %   This function is simply a projection from the state space 
            %   of the agents into the position space.
            
            value = obj.state(1:3);
        end
        
        function value = get.velocity(obj)
            %GET.VELOCITY Implementation of the dependent velocity
            %property.
            %   The linear velocity terms in the state vector of the model
            %   are given in the body-fixed reference frame. In order for
            %   the velocities of different agents to be comparable, this
            %   velocity is transformed into the inertial frame first.
            
            R     = body_frame_rotation(obj.state(4:6));
            value = R * obj.state(7:9);
        end
    end
    
    methods(Access = protected)
        function move(obj, u)
            %MOVE Calling this function once takes the agent forward in
            %time by one time step.
            %   This method is overridden from the DynamicAgent class to
            %   provide saturation effects for the HippoCampus. Without
            %   this override it would not be possible to save the
            %   unsaturated controls for inspection.
            
            if nargin >= 2
                obj.u_sat = u;
                
                if obj.saturation
                    obj.move@DynamicAgent(saturate_controls(u));
                else
                    obj.move@DynamicAgent(u);
                end
            else
                obj.move@DynamicAgent();
            end
        end
    end
    
    methods(Static, Sealed, Access = private)
        function xdot = odeFun(~, x, in)
            %ODEFUN ODE representation of the HippoCampus AUV
            %   The HippoCampus is modelled as a 12th order dynamic system.
            %   Its states are:
            %   eta = [N, E, D, phi, theta, psi]'
            %   nu  = [u, v, w, p, q, r]'
            %   x   = [eta', nu']'
            
            %   The input vector in is composed of four signals as such:
            %   in = [f, tau_phi, tau_theta, tau_psi]'

            % Extract states into more readable form
            phi = x( 4); theta = x( 5);
            u   = x( 7); v     = x( 8); w   = x( 9);
            p   = x(10); q     = x(11); r   = x(12);
            nu  = x(7:12);
            
            % Construct rotational tensor
            R = body_frame_rotation(x(4:6));
            
            % Construct velocity transformation tensor
            T = [ 1  sin(phi)*tan(theta)   cos(phi)*tan(theta) ;
                  0  cos(phi)             -sin(phi)            ;
                  0  sin(phi)/cos(theta)   cos(phi)/cos(theta) ];
            J = [ R         zeros(3) ;
                  zeros(3)  T        ];
            
            % Calculate mass matrix
            Ma  = -diag([HippoCampus.Xudot, HippoCampus.Yvdot, HippoCampus.Zwdot,...
                         HippoCampus.Kpdot, HippoCampus.Mqdot, HippoCampus.Nrdot]);
            Mrb =  diag([HippoCampus.m,  HippoCampus.m,  HippoCampus.m,...
                         HippoCampus.Ix, HippoCampus.Iy, HippoCampus.Iz]);
            M = Ma + Mrb;
            
            % Compute hydrostatic load
            gravity = HippoCampus.zg * HippoCampus.g * HippoCampus.m;
            G = [ 0; 0; 0; gravity * cos(theta) * sin(phi); gravity * sin(theta); 0 ];
            
            % Construct coriolis matrix, see Fossen 2011, Theorem 3.2
            C12 = -skew(M(1:3,1:3) * nu(1:3));
            C = [ zeros(3)   C12                        ;
                  C12       -skew(M(4:6,4:6) * nu(4:6)) ];
            
            % Construct damping matrix
            D = -diag([HippoCampus.Xu * abs(u), HippoCampus.Yv * abs(v),...
                       HippoCampus.Zw * abs(w), HippoCampus.Kp * abs(p),...
                       HippoCampus.Mq * abs(q), HippoCampus.Nr * abs(r)]);
            
            % Build state space equations
            tau  = [ in(1); 0; 0; in(2:4) ];
            
            xdot       = zeros(12,1);
            xdot(1:6)  = J*nu;
            xdot(7:12) = M \ (tau - (C+D)*nu - G);
        end
    end
end

function S = skew(lambda)
%SKEW Implements the skew function, as defined in Fossen 2011, Def. 2.2
%   This function constructs a matrix that can be used to calculate a cross
%   product between vectors: a x b = S(a)b

    S = [  0          -lambda(3)   lambda(2) ;
           lambda(3)   0          -lambda(1) ;
          -lambda(2)   lambda(1)   0         ]; 
end

function R = body_frame_rotation(Phi)
%BODY_FRAME_ROTATION Constructs the rotational matrix that transforms a
%vector from the rotated body frame of the HippoCampus into the inertial
%reference frame.

    phi = Phi(1); theta = Phi(2); psi = Phi(3);

    R = [  cos(psi)*cos(theta)  cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi)  cos(psi)*cos(phi)*sin(theta)+sin(psi)*sin(phi) ;
           sin(psi)*cos(theta)  sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi)  sin(psi)*cos(phi)*sin(theta)-cos(psi)*sin(phi) ;
          -sin(theta)           cos(theta)*sin(phi)                             cos(theta)*cos(phi)                            ];
end

function u_sat = saturate_controls(u)
    %SATURATE_CONTROLS Saturates the control signals to the limits
    %specified in the HippoCampus model.
    %   The control inputs for the model are given as the force in
    %   x direction and torques around the center of gravity.
    %   However, the saturation takes place in terms of the
    %   controls of the four rotors. Therefore the control inputs
    %   have to be first converted to the rotor signals, saturated
    %   in that form, and then converted back.

    % This is not the real saturation function, just a dummy for testing!
    val = 2*max(0.5, norm(u));
    u_sat = u / val;
end

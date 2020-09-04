% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef HippoCampusAgent < HippoCampus
    %HIPPOCAMPUSAGENT Agent implementation with a dynamic unicycle model.
    %   These agents will perform a formation control maneuvre, where agent
    %   1 acts as a leader and will guide the formation along a pre-defined
    %   reference trajectory.
    
    properties(Constant)
        sampleFactor  = 33 % Factor between the sampling rates of outer and inner controller
    end
    
    properties(GetAccess = public, SetAccess = private)
        ref
        f     % Applied force, output of outer controller
        att_d % Desired attitude, output of outer controller
    end
    
    properties(GetAccess = private, SetAccess = private)
        skipCounter % Counter of samples to be skipped till next outer controller update
        t           % Current time of the simulation
    end
    
    properties(GetAccess = private, SetAccess = immutable)
        Kin  % Inner controller
        Kout % Outer controller
    end
    
    methods
        function obj = HippoCampusAgent(id, initialPos)
            %HIPPOCMAPUSAGENT Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent and consensus protocol to the given initial position.
            
            % Set the desired sampling Time for the inner loop. The outer
            % controller will be sampled sampleFactor times slower.
            dT = 1e-3;
            dTout = dT * HippoCampusAgent.sampleFactor;
            
            % Load data from synthesis script
            data = load('HippoCampusController');
            obj@HippoCampus(id, dT, initialPos);
            
            % Initialize internal state of the simulation model
            obj.ref   = zeros(3,1);
            obj.att_d = zeros(3,1);
            obj.f     = 0;
            obj.t     = 0;
            
            % Build inner controller
            % The vertex controllers are combined by mutli-dimensional
            % linear interpolation (mdlerp).
            Ain = @(rho) mdlerp(data.Ain, data.DomIn, rho);
            Bin = @(rho) mdlerp(data.Bin, data.DomIn, rho);
            Cin = @(rho) mdlerp(data.Cin, data.DomIn, rho);
            Din = @(rho) mdlerp(data.Din, data.DomIn, rho);
            obj.Kin = ContinuousLpvDynamics(dT, [], Ain, Bin, Cin, Din, zeros(12,1));

            % Build outer controller
            % The vertex controllers are combined by mutli-dimensional
            % linear interpolation (mdlerp).
            Aout = @(rho) mdlerp(data.Aout, data.DomOut, rho);
            Bout = @(rho) mdlerp(data.Bout, data.DomOut, rho);
            Cout = @(rho) mdlerp(data.Cout, data.DomOut, rho);
            Dout = @(rho) mdlerp(data.Dout, data.DomOut, rho);
            obj.Kout = ContinuousLpvDynamics(dTout, [], Aout, Bout, Cout, Dout, zeros(12,1));
            
            % Always evaluate outer controller in the first step
            obj.skipCounter = 1;
        end

        function step(obj)
            % The HippoCampus should follow a figure 8 trajectory
            obj.ref = [ 4*sin(2*pi*obj.t/100) ;
                        8*sin(2*pi*obj.t/200) ;
                        0                     ];
            
            obj.skipCounter = obj.skipCounter - 1; 
            if obj.skipCounter == 0
                % Evaluate outer controller
                rhoOut = obj.state(5:9); % theta, psi, u, v, w
                eOut   = obj.ref - obj.position; % N, E, D
                vals   = obj.Kout.step(eOut, rhoOut); % f, theta, psi
                
                % Split and save output for the next iterations
                obj.f          = vals(1);
                obj.att_d(2:3) = vals(2:3); % phi is assumed to be 0
                
                % Refresh counter
                obj.skipCounter = obj.sampleFactor;
            end
            
            % Evaluate inner controller
            rhoIn = obj.state([5, 10:12]); % theta, p, q, r
            eIn   = obj.att_d - obj.state(4:6); % phi, theta, psi
            tau   = obj.Kin.step(eIn, rhoIn);
            
            % Apply control inputs to the agent
            obj.move([obj.f; tau]);
            obj.t = obj.t + obj.dT;
        end
    end
end
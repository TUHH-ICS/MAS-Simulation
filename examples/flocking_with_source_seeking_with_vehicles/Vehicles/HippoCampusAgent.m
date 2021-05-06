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
        K_p_att =diag([4, 4, 4]);
        K_d_att = diag([1.5, 2, 2]);
        
        K_p_pos = 1;
        K_d_pos = 1;
    end
    
    methods
        function obj = HippoCampusAgent(id, initialPos)
            %HIPPOCMAPUSAGENT Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent and consensus protocol to the given initial position.

            obj@HippoCampus(id, 0.005, initialPos, false);            
            
        end

        function step(obj,ref)            
            % Calculate control error
            e = obj.position - ref;
            
            % Implement linear thrust controller
            vel = -HippoCampusAgent.K_p_pos * norm(e);
            f   =  HippoCampusAgent.K_d_pos * (vel - obj.state(7));
            % Note that the controller is designed to move them backwards.
            % So, the velocities will most of the times be negative
            
            % Implement SO(3) attitude controller
            pitch = -atan2(e(3), norm(e(1:2)));
            yaw   =  atan2(e(2), e(1));
            Phi   = [ 0; pitch; yaw ; zeros(3,1) ];
            tau   = attitude_controller(HippoCampusAgent.K_p_att,...
                                        HippoCampusAgent.K_d_att,...
                                        obj.state([4:6, 10:12]), Phi);
            
            % Apply control inputs to the agent
            obj.move([f; tau]);            
        end
    end
end
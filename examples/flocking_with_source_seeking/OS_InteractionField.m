classdef OS_InteractionField
    %Interactionction field between agents 
    % This object defines the ineraction field between agents as defined
    % by [Olfati Saber, 2004]
   properties(Constant)
        epsilon         = 0.1; % Used to define the sigma_norm
        da              = 7;   % Equilibrium distance(or desired distance) to neighbours
        ra              = 1.2 * OS_InteractionField.da; % Sensing radius (no interaction with agents outside ra disc)
        h               = 0.9; % Bump function goes to zero after h
   end
   properties(GetAccess = public, SetAccess = private)
        psi_a_data      % Numerically integrate the force to obtain the potential (Look-up)
        
    end
   methods
        function obj = OS_InteractionField()
            % Constructor function for Initializing interaction field 

            % Store psi_a(Lookup) for avoiding numerical integration at each step
            % This function numerically integrates the phi=d(psi)/dx to get psi.
            % The interaction potential psi does not have a closed form solution and
            % needs to be computed by numerical integration. To avoid numerical
            % integration each time, this function stores the input output data for 
            % interaction potential psi.  
            dz=0.01;
            z=0:dz:10;
            for i=1:size(z,2)
                vec(i)=psi_a(z(i), obj);
            end
            psi_a_data=struct;
            psi_a_data.output=vec;
            psi_a_data.input=0:dz:10;
            obj.psi_a_data=psi_a_data;
        end
        function [force,a]=get_interaction_force(obj,dist_euclidean)
            % This function takes in the Euclidean distance and outputs the
            % interaction force based on the field parameters and the
            % adjacency element governing the topology.
            
            % Compute sigma distance for a given euclidean distance
            [dist, grad] = sigma_norm(dist_euclidean,obj.epsilon);
            % Compute the interaction force between for a given distance
            force=grad * phi_alpha(dist, obj.ra,obj.da, obj.h);  
            % Compute the adjacency element for a given distance
            a = rho_h(dist / obj.ra, obj.h);  
        end
        function psi_eval=look_up_psi_a(obj,z)
            u=obj.psi_a_data.input;
            y=obj.psi_a_data.output;
            
            [val,I]=max(z<u,[],2);
            
            psi_eval(find(val~=0))=y(I(find(val~=0)));
            psi_eval(val==0)=y(end); 
            
        end        
   end
end
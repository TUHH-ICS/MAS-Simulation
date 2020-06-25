classdef InvGaussiansField
    %field_inv_gaussians 
    % This object defines an underlying scalar field as a sum of inverted
    % Gaussian functions at different centers with heterogeneous variances
    % and scaling.
    % Could define a Baseclass called field to this class to generalize to
    % other fields
   properties
      name              % Name of the field
      dim               % spatial dimensions agents live in
      no_centers        % No of minima in the field
      centers           % Locations of minima
      Sigmas            % Co-variance matrices at each center
      scales            % scaling at each center
   end
   methods
        function obj = InvGaussiansField(d,no_centers,fcenter,frange,fvar,fscale)
            % Constructor function for setting the properties
            obj.name       = 'Sum_inv_gaussians';
            obj.dim        = d;
            obj.no_centers = no_centers; % No of Gaussians
            obj.centers    = fcenter+frange*(-0.5+rand(d,no_centers));
            obj.Sigmas     = kron(ones(1,no_centers),fvar*eye(d)); 
            obj.scales     = fscale*ones(1,no_centers);
        end
        function field_eval=get_field_value_at(obj,z)
            % This function outputs the value of the field at the 
            % location z
            field_eval=zeros(1,size(z,2));            
            for i=1:obj.no_centers
                source_i=obj.centers(:,i);
                Sigma_i=obj.Sigmas(:,(i-1)*obj.dim+1:i*obj.dim);
                scale_i=obj.scales(i);
                e=(z-source_i);
                field_eval = field_eval +scale_i*exp(-0.5*diag(e'*inv(Sigma_i)*e)');
            end
            field_eval=-1*field_eval;
        end
        function grad = get_true_gradient(obj,q_agent) 
            %GET.GRAD_ESTIMATE 
            %   This function asks for a gradient at the q_agent location. 
            %   Can be thought of as a gradient measurement            
            grad=zeros(obj.dim,1);            
            for i=1:obj.no_centers
                source_i=obj.centers(:,i);
                Sigma_i=obj.Sigmas(:,(i-1)*obj.dim+1:i*obj.dim);
                scale_i=obj.scales(i);
                e=(q_agent-source_i);
                grad = grad +scale_i*exp(-0.5*e'*inv(Sigma_i)*e)*inv(Sigma_i)*e;
            end
        end
        % Should also define a get_true_hessian function later
   end
end
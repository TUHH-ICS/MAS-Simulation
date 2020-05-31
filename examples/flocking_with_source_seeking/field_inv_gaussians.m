classdef field_inv_gaussians
    %field_inv_gaussians 
    % This object defines an underlying scalar field as a sum of inverted
    % Gaussian functions at different centers with heterogeneous variances
    % and scaling.
    % Could define a Baseclass called field to this class to generalize to
    % other fields
   properties
      name              % Name of the field
      no_of_minima      % No of minima in the field
      centers           % Locations of minima
      var               % variance at each center
      scale             % scaling at each center
   end
   methods
        function obj = field_inv_gaussians(params)
            % Constructor function for setting the properties
            if nargin == 1
                obj.name                = params.name;
                obj.no_of_minima        = params.invt_gauss_no_minima;
                obj.centers             = params.invt_gauss_center_m;
                obj.var                 = params.invt_gauss_var_m;
                obj.scale               = params.invt_gauss_scale_m;
            end
        end
        function grad = get_true_gradient(obj,q_agent) 
            %GET.GRAD_ESTIMATE 
            %   This function asks for a gradient at the q_agent location. 
            %   Can be thought of as a gradient measuremen            
            grad=zeros(size(q_agent,1),1);
            for i=1:obj.no_of_minima
                source_i=obj.centers(:,i);
                var_i=obj.var(:,i);
                scale_i=obj.scale(:,i);
                error_source=(q_agent-source_i);
                grad = grad +scale_i*exp(-norm(error_source)^2/var_i)*error_source;
            end
        end
        % Should also defined a get_true_hessian function later
   end
end
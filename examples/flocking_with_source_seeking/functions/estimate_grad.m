function grad=estimate_grad(q_agent,params)
% This function returns an estimate of the gradient of the scalar field 
% at the current agent location

% Currently just providing the exact gradient plus noise
% This can be replaced by querring 10 nearby points and solving a least
% squares problem to estimate the gradient


% Sum of inverted Gaussians with a multiple minima

no_minima=params.invt_gauss_no_minima;
grad=zeros(2,1);
for i=1:no_minima
    source=params.invt_gauss_center_m(:,i);
    var=params.invt_gauss_var_m(:,i);
    scale=params.invt_gauss_scale_m(:,i);
    error_source=(q_agent-source);
    grad = grad +scale*exp(-norm(error_source)^2/var)*error_source;
end

end
function source_seek_params = source_seeking_parameters()
%SOURCE_SEEKING_PARAMETERS Function that returns a parameter structure with all
%relevant quantities set to the desired value

source_seek_params=struct;

% Inverted_gaussian with multiple minima
source_seek_params.name                   ='Inverted_Gaussian_multiple_minima';
source_seek_params.invt_gauss_no_minima   =11;

% generate random centers in the box [0,100]^2
source_seek_params.invt_gauss_center_m    =[];
for i=1:source_seek_params.invt_gauss_no_minima
center=50+100*(-0.5+rand(2,1));
source_seek_params.invt_gauss_center_m    =[source_seek_params.invt_gauss_center_m,center];
end

% variance and scaling at each center
source_seek_params.invt_gauss_var_m       =1e2*ones(1,source_seek_params.invt_gauss_no_minima);
source_seek_params.invt_gauss_scale_m     =1*ones(1,source_seek_params.invt_gauss_no_minima);

end
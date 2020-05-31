function [X,Y,Z]=data_for_contour(lim,source_seek_params)
% This function creates mesh grid data for plotting contours based on the
% underlying source field
% Sum of Inverted Gaussians
no_minima=source_seek_params.invt_gauss_no_minima;
XX = lim(1,1):1:lim(1,2);
YY = lim(2,1):1:lim(1,2);    
[X,Y] = meshgrid(XX,YY);
Z=zeros(size(X));
for i=1:no_minima
    source=source_seek_params.invt_gauss_center_m(:,i);
    var=source_seek_params.invt_gauss_var_m(:,i);
    scale=source_seek_params.invt_gauss_scale_m(:,i);
    Z = Z +scale*exp(-((X-source(1)).^2+(Y-source(2)).^2)/var);
end 

end
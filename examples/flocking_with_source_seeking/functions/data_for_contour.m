% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Adwait Datar <adwait.datar@tuhh.de>

function [X,Y,Z]=data_for_contour(lim,inv_gauss_field)
% This function creates mesh grid data for plotting contours based on the
% underlying source field
% Sum of Inverted Gaussians
no_minima=inv_gauss_field.no_centers;
d=inv_gauss_field.dim;
XX = lim(1,1):1:lim(1,2);
YY = lim(2,1):1:lim(2,2);    
[X,Y] = meshgrid(XX,YY); 
Z=zeros(size(X));
for i=1:no_minima
    Source_i=inv_gauss_field.centers(:,i);
    Sigma_i=inv_gauss_field.Sigmas(1:d,(i-1)*d+1:i*d);
    scale_i=inv_gauss_field.scales(i);
    Ex=[XX-Source_i(1)];
    Ey=[YY-Source_i(2)];
    Z_temp=zeros(size(X));
    for m=1:size(YY,2)
        for n=1:size(XX,2)
            e=[Ex(n);Ey(m)];
            Z_temp(m,n)=scale_i*exp(-0.5*e'*inv(Sigma_i)*e);
        end
    end
    Z=Z+Z_temp;
    end 

end
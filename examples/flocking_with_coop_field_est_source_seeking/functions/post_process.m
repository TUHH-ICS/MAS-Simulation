% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Adwait Datar <adwait.datar@tuhh.de>

function []=post_process(dT,timesteps,Energy)
% This function post-processes the data collected during simulation
V=Energy.V;
Vfield=Energy.Vfield;
KE=Energy.KE;

t=dT*[1:timesteps];

Vn_inter=V-min(V);
Vn_kin=KE-min(KE);
Vn_field=Vfield-min(Vfield);
Vn_kin_plus_inter=Vn_inter+Vn_kin;

Vn_total_potential=Vn_inter+Vn_field;
Vn_total=Vn_total_potential+Vn_kin_plus_inter;

figure()
plot(t,Vn_inter)
hold on
plot(t,Vn_kin)
plot(t,Vn_kin_plus_inter)
legend('V interaction','KE','KE + V interaction')

figure()
plot(t,Vn_field)
title('External potential energy')

figure()
plot(t,Vn_total_potential)
hold on
plot(t,Vn_kin)
plot(t,Vn_total)
legend('Total potential','Kinetic Energy','Total energy')

end
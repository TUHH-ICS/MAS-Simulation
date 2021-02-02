% This file takes the gridded LPV controllers synthesized by the LPVTools
% library and extracts the vertex controllers. In that form the controllers
% can be used in the simulation library.

% Convert the inner loop controller into the desired format
[Ain_p, Bin_p, Cin_p, Din_p] = ssdata(Kin);
DomIn = Kin.Domain.IVData;
Ain = double(Ain_p);
Bin = double(Bin_p);
Cin = double(Cin_p);
Din = double(Din_p);

% Convert the outer loop controller into the desired format
[Aout_p, Bout_p, Cout_p, Dout_p] = ssdata(Kout);
DomOut = Kout.Domain.IVData;
Aout = double(Aout_p);
Bout = double(Bout_p);
Cout = double(Cout_p);
Dout = double(Dout_p);

% Save both controller into the .mat file
save('HippoCampusController.mat', 'DomIn',  'Ain',  'Bin',  'Cin',  'Din',...
                                  'DomOut', 'Aout', 'Bout', 'Cout', 'Dout');
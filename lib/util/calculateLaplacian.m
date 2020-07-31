% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

function L = calculateLaplacian(recvMessages)
%CALCULATELAPLACIAN Function that calculates the Laplacian matrix based on
%the received messages and the sender information contained in the messages.

L = diag(cellfun(@length, recvMessages));
for i = 1:length(recvMessages)
    L(i,[recvMessages{i}.sender]) = -1;
end
end

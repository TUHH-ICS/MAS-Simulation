%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

classdef SinrConfiguration < matlab.mixin.Copyable
    %SINRCONFIGURATION Class that bundles all configuration options of the
    %SinrNetwork.
    
    properties
        % Number of agents that communicate over the network
        agentCount(1,1) {mustBeInteger, mustBePositive, mustBeLessThanOrEqual(agentCount, 1000)} = 5
        
        % Number of slots that are available for the agents to communicate
        slotCount(1,1) {mustBeInteger, mustBeFinite, mustBePositive} = 5
        
        % Duration of one sending cycle
        cycleTime(1,1) {mustBeFinite, mustBePositive} = 1
        
        % Size of the data packets [Bit]
        packetSize(1,1) {mustBeInteger, mustBeFinite, mustBePositive} = 6 * 64;
        
        % Temperature of the environment [K]
        temperature(1,1) {mustBeFinite, mustBePositive} = 293.15 % 20° C
        
        % Wireless protocol that is used for the transmission
        wirelessProtocol(1,1) WirelessProtocol = WirelessProtocol.wp_802_11n_mode_1
        
        % Transmission power [W]
        power(1,1) {mustBeFinite, mustBePositive} = 1
        
        % Path loss coefficient of the transmission
        pathLoss(1,1) {mustBeFinite, mustBeGreaterThanOrEqual(pathLoss, 2)} = 2.0
        
        % Parameter of the Nakagami distribution used for the fading model
        nakagamiParameter(1,1) {mustBeFinite, mustBeGreaterThanOrEqual(nakagamiParameter, 0.5)} = 2.0
        
        % If true, simulateneously received packages get drop, otherwise
        % there is a random choice between all of them.
        dropCollisions(1,1) {mustBeA(dropCollisions, "logical")} = false
        
        % If true, a BPSK model is used for the modulation scheme,
        % otherwise the simulation assumes idealized transmission.
        bpsk(1,1) {mustBeA(bpsk, "logical")} = true
        
        % Seeds for the random number generators
        fadingSeed(1,1) uint32 = randi(intmax('uint32'))
        slotSeed(1,1) uint32 = randi(intmax('uint32'))
    end
    
    properties(Dependent)
       slotTime(1,1) {mustBeFinite, mustBePositive} 
    end
    
    methods
        function value = get.slotTime(obj)
            %GET.SLOTTIME Getter implementation of the dependent property
            %slotTime.
            
            value = obj.cycleTime / obj.slotCount;
        end
        
        function set.slotTime(obj, value)
            %SET.SLOTTIME Setter implementation of the dependent property
            %slotTime.
            
            obj.cycleTime = obj.slotCount * value;
        end
    end
end

%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

classdef WirelessProtocol
    %WIRELESSPROTOCOL Enumeration that lists all wireless protocols that
    %are defined in the C++ library
    
    enumeration
        wp_802_11n_mode_1   % N-Standard WLAN with low bitrate
        wp_802_11n_mode_2   % N-Standard WLAN with high bitrate
        wp_802_11p          % Standard for car to car communication
        wp_802_15_4_europe  % Standard for wireless sensor networks
        
        % cf. "Underwater Acoustic Modems", Sandra Sendra et al., IEEE Sensors Journal, 2016.
        underwater_mako_modem   % 240 bits/s bitrate
        underwater_marlin_modem % 480 bits/s bitrate
    end
end

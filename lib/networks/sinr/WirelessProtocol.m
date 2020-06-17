classdef WirelessProtocol
    %WIRELESSPROTOCOL Enumeration that lists all wireless protocols that
    %are defined in the C++ library
    
    enumeration
        wp_802_11n_mode_1   % N-Standard WLAN with low bitrate
        wp_802_11n_mode_2   % N-Standard WLAN with high bitrate
        wp_802_11p          % Standard for car to car communication
        wp_802_15_4_europe  % Standard for wireless sensor networks
    end
end


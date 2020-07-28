#pragma once

#include <vector>
#include <tuple>
#include <string>
#include <random>
#include "tools.h"

 /* *****************************************************
 *  fading.h
 *  Author: Daniel Schneider [ds] (schneiderd@uni-koblenz.de)
 *  Date: 2020-06-09
 *
 *  Comments:
 *
 *  *****************************************************
*/                

struct _object;
typedef _object PyObject;

namespace SINR{ // Namespace SINR

typedef unsigned int AgentID;
typedef unsigned int SlotNumber;
//struct vec3;
using TLS::vec3;
//typedef const unsigned int SlotNumber;

class WirelessProtocolParameters;

class Fading{
	public:
		typedef std::vector<double> PowerVec;
		typedef double FadingValue;
		typedef std::tuple<AgentID, FadingValue> t2_AgentIDFadingValue;

		const unsigned int m_numberOfAgents;
		//double* m_interferenceByAgent;
		std::default_random_engine* m_pRandomGenerator;
		const unsigned int m_randomSeed;
		PowerVec& m_powerVec;
		WirelessProtocolParameters& m_protocol;
		const double m_alphaPL;
		double** m_interferenceInCurrentSlot;

		//Fading(const unsigned int numberOfAgents, const unsigned int randomSeed, WirelessProtocolParameters& protocol);
		Fading(const unsigned int numberOfAgents, const unsigned int randomSeed, PowerVec& powerVector, WirelessProtocolParameters& protocol, const double alphaPL);
		virtual ~Fading() = 0;
		//virtual std::vector<t2_AgentIDFadingValue> sample(std::vector<AgentID> agentIDs);
		virtual void sample_by_slot(const std::vector<AgentID>& sendingAgents, const vec3** pAgentPositions);

		double get_mu_ij(const AgentID& i, const AgentID& j, const vec3** pAgentPositions);
}; 


class NakagamiFading : public Fading{
	public:
		const double m_nakagamiParam;

		NakagamiFading(const unsigned int numberOfAgents, const unsigned int randomSeed, PowerVec& powerVector, WirelessProtocolParameters& protocol, const double alphaPL, const double nakagamiParam);
		~NakagamiFading();

		void sample_by_slot(const std::vector<AgentID>& sendingAgents, const vec3** pAgentPositions);

};

class WirelessProtocolParameters{
	public:
		const double m_freq;
		const double m_channelCapacity;
		const double m_channelBandwidth;
		const double m_temperature;
		const double m_c;
		const double m_boltzmannConst;
		double m_bitrate;


		WirelessProtocolParameters(const double freq, const double channelCapacity, const double channelBandwidth, const double temperature, const double c = 299792458) : m_freq(freq), m_channelCapacity(channelCapacity), m_channelBandwidth(channelBandwidth), m_temperature(temperature), m_c(c), m_boltzmannConst(1.38064852e-23), m_bitrate(channelCapacity){};
		virtual ~WirelessProtocolParameters() = 0;

 		virtual double adapt_power(double power);
		virtual double path_loss(const vec3& u, const vec3& v, const double pathLossCoeff); 
		virtual double noise();
		virtual double wavelength();
		virtual double sinr_threshold();
		virtual double get_far_field_distance(const double antennaDimension, const unsigned int k = 5);
		virtual void set_bitrate(const double& bitrate);

};

class WirelessProtocol802_11n_mode1 : public WirelessProtocolParameters{
    /* 802.11 n
    
    :param mode: 
    :type mode: integer

    :param temp: Temperature [K]
    */

public:
	WirelessProtocol802_11n_mode1(const double temperature) :  WirelessProtocolParameters(
        2.4e9,   // 2.4 GHz
        288.8e6, // 288.8 Mbit/s
        20e6,     // 20 MHz
        temperature
	){}; 
};
class WirelessProtocol802_11n_mode2 : public WirelessProtocolParameters{
    /* 802.11 n
    
    :param temp: Temperature [K]
    */

public:
	WirelessProtocol802_11n_mode2(const double temperature) :  WirelessProtocolParameters(
        2.4e9,   // 2.4 GHz
        600e6,   // 600 Mbit/s
         40e6,   // 40 MHz
        temperature
	){}; 
};

class WirelessProtocol802_11p: public WirelessProtocolParameters{
    	/*802.11 p */

public:
    WirelessProtocol802_11p(const double temperature= 293 /* 20°C*/) :  WirelessProtocolParameters(
        5.9e9,  // 5.9 GHz
        27.0e6, // 27 Mbit/s
        10e6,   // 10 MHz
        temperature 

    ){};
};

class WirelessProtocol802_15_4_europe : public WirelessProtocolParameters{

public:
	WirelessProtocol802_15_4_europe(const double temperature) : WirelessProtocolParameters(
        868e6, 				// 868 Mhz
        20e3, 				// 20 kbit/s
        (868.6 - 868.0)/1.0 * 1e6,   	// 600 kHz;
        temperature
	){}; 

};

struct UnderwaterModem{
	const double bitrate;
};

extern UnderwaterModem Mako;
extern UnderwaterModem Marlin;

class UnderwaterProtocol : public WirelessProtocolParameters {
    /*
    :param modem: one of (SINR::)Mako, (SINR::)Marlin
    :param shippingParameter: shipping activity between 0 and 1
    :param windParameter: wind speed in m/s
    */

    public:
	    const double m_shippingParameter;
	    const double m_windParameter;

	    // cf. "Underwater Acoustic Modems", Sandra Sendra et al., IEEE Sensors Journal, 2016.
	    UnderwaterProtocol(const UnderwaterModem& model, const double shippingParameter = 0.5, const double windParameter = 10.0) : WirelessProtocolParameters(
				     23e3, // frequency = 23 kHz
				     model.bitrate, // bitrate = 240 bit/s (Mako) or 480 bit/s (Marlin)
				     14e3,   // channel bandwith = 14 kHz 
				     0, // temperature is not used??
				     1500.0 // c, i.e. speed of sound underwater [m/s]
			      ), m_shippingParameter(shippingParameter), m_windParameter(windParameter){}; 


	    double adapt_power(double power);
	    double noise();
	    double path_loss(const vec3& u, const vec3& v, const double pathLossCoeff);

}; 


} // end namespace SINR

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

		double noise();
		double wavelength();
		double sinr_threshold();
		double get_far_field_distance(const double antennaDimension, const unsigned int k = 5);
		void set_bitrate(const double& bitrate);

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
} // end namespace SINR

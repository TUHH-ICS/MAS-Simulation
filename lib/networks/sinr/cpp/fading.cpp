#define _USE_MATH_DEFINES

#include "fading.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <assert.h>
#include <random>
#include <set>

/* *****************************************************
 *  fading.cpp
 *  Author: Daniel Schneider [ds] (schneiderd@uni-koblenz.de)
 *  Date: 2020-06-09
 *
 *  *****************************************************
*/

namespace SINR{

// ############## BEGIN: Fading ##############

Fading::Fading(const unsigned int numberOfAgents, const unsigned int randomSeed, PowerVec& powerVector, WirelessProtocolParameters& protocol, const double alphaPL) : m_numberOfAgents(numberOfAgents), m_randomSeed(randomSeed), m_powerVec(powerVector), m_protocol(protocol), m_alphaPL(alphaPL){
	m_pRandomGenerator = new std::default_random_engine(randomSeed);

	m_interferenceInCurrentSlot = new double*[m_numberOfAgents];
	for (unsigned int i = 0; i < m_numberOfAgents; i++){
		m_interferenceInCurrentSlot[i] = new double[m_numberOfAgents];
		for (unsigned int j = 0; j < m_numberOfAgents; j++){
			m_interferenceInCurrentSlot[i][j] = 0.0;
		} 
	}
}

Fading::~Fading(){
	delete m_pRandomGenerator;

	for (unsigned int i = 0; i < m_numberOfAgents; i++){
		delete[] m_interferenceInCurrentSlot[i];
	}

	delete[] m_interferenceInCurrentSlot;
}; 

void Fading::sample_by_slot(const std::vector<AgentID>& sendingAgents, const vec3** pAgentPositions){
	std::cerr << "was Here base!!!!" << std::endl;
	exit(1);
} 

double Fading::get_mu_ij(const AgentID& i, const AgentID& j, const vec3** pAgentPositions){

			/*return std::pow(TLS::vec3_dist(
					*(pAgentPositions[i]),
					*(pAgentPositions[j])) * 4.0 * M_PI /m_protocol.wavelength(), m_alphaPL) /m_powerVec[i];
			*/
			return m_protocol.path_loss(*(pAgentPositions[i]), *(pAgentPositions[j]), m_alphaPL)/m_protocol.adapt_power(m_powerVec[i]) * std::pow(4.0 * M_PI /m_protocol.wavelength(), m_alphaPL);

}


// ############## END: Fading ##############


 
// ############## BEGIN: Nakagami Fading ##############


NakagamiFading::NakagamiFading(const unsigned int numberOfAgents, const unsigned int randomSeed, PowerVec& powerVector, WirelessProtocolParameters& protocol, const double alphaPL, const double nakagamiParam): Fading(numberOfAgents, randomSeed, powerVector, protocol, alphaPL), m_nakagamiParam(nakagamiParam){
}

NakagamiFading::~NakagamiFading(){
	//delete m_pNFHelper;
}

void NakagamiFading::sample_by_slot(const std::vector<AgentID>& sendingAgents, const vec3** pAgentPositions){		
	const unsigned int agentsInSlot = sendingAgents.size();
	assert(agentsInSlot >= 1);

	for (AgentID id_i: sendingAgents){
		for (AgentID id_j = 0; id_j < m_numberOfAgents; id_j++){
			if (id_i == id_j)
				{continue;} 	
			const double mu_ij = get_mu_ij(id_i, id_j, pAgentPositions);

			// gamma distribution with shape and scale parameters as first and second argument resp.
			// These are called *k* and *theta* in the English Wikipedia as of 2020-02-20.
			const double k = m_nakagamiParam;
			const double theta = 1.0/(mu_ij * m_nakagamiParam);
     			std::gamma_distribution<> gamma(k,theta);
			m_interferenceInCurrentSlot[id_i][id_j] = gamma(*m_pRandomGenerator);
		}
	} 
} 

// ############## END: Nakagami Fading ##############

// ############## BEGIN: WirelessProtocolParameters ##############

double WirelessProtocolParameters::path_loss(const vec3& u, const vec3& v, const double pathLossCoeff){
	return std::pow(TLS::vec3_dist(u,v), pathLossCoeff);
}

double WirelessProtocolParameters::noise(){
	/*Returns noise in [W]*/

	return m_boltzmannConst * m_temperature * m_channelBandwidth;
}

double WirelessProtocolParameters::wavelength(){
	return m_c / m_freq;
}

double WirelessProtocolParameters::sinr_threshold(){
	return std::pow(2.0, (m_bitrate/m_channelBandwidth)-1.0);
}

double WirelessProtocolParameters::get_far_field_distance(const double antennaDimension, const unsigned int k){
	const double D = antennaDimension;

	return std::max(k * D, std::max(k * wavelength(), 2.0* D*D/wavelength()));
}

void WirelessProtocolParameters::set_bitrate(const double& bitrate){
	m_bitrate = bitrate;
} 

double WirelessProtocolParameters::adapt_power(double power){
	// Here in the base class, we just use the identity.
	return power;
} 

//
// ############## END: WirelessProtocolParameters ############## 

// ############## BEGIN: UnderwaterProtocol ##############

// Define underwater modems
// cf. "Underwater Acoustic Modems", Sandra Sendra et al., IEEE Sensors Journal, 2016.
UnderwaterModem Mako{240.0};   //240 bits/s
UnderwaterModem Marlin{480.0}; //480 bits/s 
 
double UnderwaterProtocol::noise(){
        /*Returns noise in [W]
        
        See Short-Range Underwater Acoustic Communication Networks, p.185
        
        */

        const double f = m_freq/1000.0;
        const double l10 = std::log(10.0);

        double N_turb =  17.0 - 30.0 * std::log(f)/l10;
        double N_ship =  40.0 + 20.0 *(m_shippingParameter -0.5) + 26.0 * std::log(f)/l10 - 60.0 * std::log(f + 0.03)/l10;
        double N_wind =  50.0 + 7.5 * std::sqrt(m_windParameter) + 20.0 * std::log(f)/l10 - 40.0 * std::log(f+0.4)/l10;
        double N_thermal = -15.0 +20.0 * log(f)/l10;

	auto my_convert = [](const double x){return std::pow(10,x/10.0);};
        N_turb = my_convert(N_turb);
        N_ship = my_convert(N_ship);
        N_wind = my_convert(N_wind);
        N_thermal = my_convert(N_thermal);
        
        return (N_turb + N_ship + N_wind + N_thermal) * m_channelBandwidth;
} 



double UnderwaterProtocol::path_loss(const vec3& u, const vec3& v, const double pathLossCoeff){
	//u has to be different from v
	assert(TLS::vec3_eq(u,v) == false);

        const double l10 = std::log(10.0);

	const double r = TLS::vec3_dist(u,v);
        const double f = m_freq/1000.0;
        double pl = pathLossCoeff * 10.0 *std::log(r)/l10 + (0.11*f*f/(1.0+f*f) + 44.0*f*f/(4100.0 + f*f) + 2.75e-4 * f*f + 0.0033) * r *1e-3; // sic: the last 10^(-3) is in the formula and has no bearing on the m <-> km calulation.
        pl = std::pow(10.0, pl/10.0);

        return pl;
}

double UnderwaterProtocol::adapt_power(double power){
        // We have SL_projector = (170.8 + 10log(P_tx)) dB, see Short-Range Underwater Acoustic Communication Networks, p.178 
        //
        
	return power * std::pow(10.0, 17.08);

}
// ############## END: UnderwaterProtocol ############## 

}// end namespace SINR

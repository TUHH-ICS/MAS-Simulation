#define _USE_MATH_DEFINES

#include "fading.h"
#include <iostream>
#include <cmath>
#include <assert.h>
#include <random>
#include <set>

//#include <python2.7/Python.h>

/* *****************************************************
 *  fading.cpp
 *  Author: Daniel Schneider [ds] (schneiderd@uni-koblenz.de)
 *  Date: 2020-06-09
 *
 *  *****************************************************
*/


namespace SINR{

//using TLS::vec3;




// ############## BEGIN: Fading ##############

Fading::Fading(const unsigned int numberOfAgents, const unsigned int randomSeed, PowerVec& powerVector, WirelessProtocolParameters& protocol, const double alphaPL) : m_numberOfAgents(numberOfAgents), m_randomSeed(randomSeed), m_powerVec(powerVector), m_protocol(protocol), m_alphaPL(alphaPL){
	//m_interferenceByAgent = new double[m_numberOfAgents];
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

	//delete[] m_interferenceByAgent;

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

			return std::pow(TLS::vec3_dist(
					*(pAgentPositions[i]),
					*(pAgentPositions[j])) * 4.0 * M_PI /m_protocol.wavelength(), m_alphaPL) /m_powerVec[i];
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


double WirelessProtocolParameters::noise(){
	/*Returns noise in [W]*/

	return m_boltzmannConst * m_temperature * m_channelBandwidth;
}

double WirelessProtocolParameters::wavelength(){
	return m_c / m_freq;
}

/*
double WirelessProtocolParameters::sinr_threshold(){
	return std::pow(2.0, (m_channelCapacity/m_channelBandwidth)-1.0);
}
*/
double WirelessProtocolParameters::sinr_threshold(){
	//assert(0<=effectiveBitrate <= m_channelCapacity);

	return std::pow(2.0, (m_bitrate/m_channelBandwidth)-1.0);
}

double WirelessProtocolParameters::get_far_field_distance(const double antennaDimension, const unsigned int k){
	const double D = antennaDimension;

	return std::max(k * D, std::max(k * wavelength(), 2.0* D*D/wavelength()));
}

void WirelessProtocolParameters::set_bitrate(const double& bitrate){
	m_bitrate = bitrate;
} 


}// end namespace SINR

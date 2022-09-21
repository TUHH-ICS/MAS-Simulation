/* -------------------------------------------------------------------------------------------------
 * Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved
 * Licensed under the GPLv3. See LICENSE in the project root for license information.
 * Author(s): Daniel Schneider
 * -------------------------------------------------------------------------------------------------
 */

#pragma once
#include <iostream>
#include <stdio.h>
#include <tuple>
#include <random>
#include <assert.h>
#include <cmath>
#include "fading.h"

// ############## BEGIN: Alias declaration ##############
typedef unsigned int AgentID;
typedef unsigned int SlotNumber;
// ############## END  : Alias declaration ############## 

using TLS::vec3;

class Data{
	public:
		vec3 m_pos;

		void init(const vec3& pos){
			m_pos = pos;	
		}
}; 

// Agent Base class
template <class TheData>
class Agent{
	public:
		TLS::Type2Type<Data> dataType;
		typedef TheData DataType;

		std::vector<std::tuple<AgentID, SlotNumber> > m_received_from;

		int set_network_time(const double t);
		static unsigned int m_IDcounter;
		const int m_ID;
		TheData* m_pData;
        bool m_sendFlag;

		Agent() : m_ID(m_IDcounter){
			m_IDcounter++;
		}
		Agent(TheData* pInitialData): m_ID(m_IDcounter), m_pData(pInitialData){
			m_IDcounter++;
		}
		virtual ~Agent() = 0;

		virtual int init(TheData* pInitialData){
		    m_pData = pInitialData;
            m_sendFlag = false;
            
			return 0;
		}
        
		vec3 get_pos();
		void set_pos(const vec3 pos);

        int set_send_flag(const bool val);
        bool get_send_flag();
        
		virtual TheData* get_data();

		virtual int pre_process();
		virtual int receive_data(AgentID sender, SlotNumber, TheData& data);
		virtual int post_process();
};
//
// ################### BEGIN class Agent ###################
template <class TheData>
Agent<TheData>::~Agent(){
} 

template <class TheData>
int Agent<TheData>::set_network_time(const double t){
	return 0;
}

template <class TheData>
vec3 Agent<TheData>::get_pos(){
	return m_pData->m_pos;
}

template <class TheData>
void Agent<TheData>::set_pos(const vec3 pos){
	m_pData->m_pos.x1 = pos.x1;
	m_pData->m_pos.x2 = pos.x2;
	m_pData->m_pos.x3 = pos.x3;
} 

template <class TheData>
TheData* Agent<TheData>::get_data(){
	return m_pData;
}

template <class TheData>
int Agent<TheData>::pre_process(){
	return 0;
}

template <class TheData>
int Agent<TheData>::receive_data(AgentID sender, SlotNumber k, TheData& data){
	std::cout << "Agent ";
	std::cout << m_ID << std::endl;
	std::cout << "is receving Data from ";
	std::cout << sender << std::endl;
	return 0;
}

template <class TheData>
int Agent<TheData>::post_process(){
    m_sendFlag = false;
	return 0;
}

template <class TheData>
int Agent<TheData>::set_send_flag(const bool val){
    m_sendFlag = val;
    return 0;
}

template <class TheData>
bool Agent<TheData>::get_send_flag(){
    // ToDo: check if *init* has been called
    return m_sendFlag;
}

//
template <class TheData>
//Must be def. outside the class and will be set to zero (Scott Meyers, effective c++, p.59).
//
unsigned int Agent<TheData>::m_IDcounter; 
// ################### END class Agent ################### 

/**
 * Enum for selecting appropriate behaviour in case of message collisions
 */
enum CollisionBehaviour {
    ReceiveAll,
    DropAll,
    PickOne,
};

double bpsk_succ_prob(const double packetSize, const double bandwidth, const double bitrate, const double sinr){

    /* packetSize: in bit
       bandwidth: corresponds to 2*B in Goldbsmith, Wireless Communication
       bitrate: in bit/s
       SINR: Signal to noise plus interference ratio

       The formula in Goldsmith reads:
        Q(sqrt(2 gamma_b))   [p.174]

    */

	double bitErrProb = TLS::qfunction(sqrt(bandwidth/bitrate * sinr));
	double succProb = pow(1.0 - bitErrProb, packetSize);

	return succProb;
} 

bool default_sinr_success_function(const double packetSize, const double bandwidth, const double bitrate, const double sinr, const double sinrThreshold, std::default_random_engine* pRandomGenerator){
      return (sinr >= sinrThreshold);
}

bool bpsk_sinr_success_function(const double packetSize, const double bandwidth, const double bitrate, const double sinr, const double sinrThreshold, std::default_random_engine* pRandomGenerator){
      std::bernoulli_distribution bernoulli(bpsk_succ_prob(packetSize, bandwidth, bitrate, sinr));
 	
      return static_cast<bool>(bernoulli(*pRandomGenerator));
}

// Simple Agent
class SomeAgent : public Agent<Data>{
	public:
		unsigned int m_numberOfAgents;
		double m_beaconFreq;

		int init(DataType* pInitialData, const unsigned int numberOfAgents, const double beaconFrequency){
 		       	m_pData = pInitialData;
			m_numberOfAgents = numberOfAgents;
			m_beaconFreq = beaconFrequency; 
			m_sendFlag = false;
			return 0;
		}
		int receive_data(AgentID sender, SlotNumber k, Data& data) override{ 
			m_received_from.push_back(std::make_tuple(sender, k));
			return 0;

		}

 		int pre_process() override{
			m_received_from.clear();
			return 0;
		}
};

//forward declaration
template <class TheData> class NetworkChannel;

template <class TheData>
class SimulationEnvironment{
	public:
	    typedef Agent<TheData> BaseAgent;
	    typedef NetworkChannel<TheData> Channel;

	    const unsigned int m_numberOfAgents;
	    std::vector<BaseAgent*> &m_pAgents;
	    const double m_samplingTime;
	    const vec3** m_pAgentPositions;
	    const CollisionBehaviour m_collisions;
	    const bool m_bpsk;

	    SimulationEnvironment(){}; 
	    SimulationEnvironment(const unsigned int numberOfAgents, std::vector<BaseAgent*>& agents, const double samplingTime, Channel& channel, const CollisionBehaviour collisions, const bool bpsk);
	    ~SimulationEnvironment();

	    void send(const AgentID sendingAgent);

	    int process();
	 protected:
	    void _send(const AgentID sendingAgent, const TheData& data);
	    Channel &m_channel;
	    bool m_inited; 
}; 

template <class MyData>
SimulationEnvironment<MyData>::SimulationEnvironment(const unsigned int numberOfAgents, std::vector<BaseAgent*>& pAgents, const double samplingTime, Channel& channel, const CollisionBehaviour collisions, const bool bpsk)  : m_numberOfAgents(numberOfAgents), m_pAgents(pAgents), m_samplingTime(samplingTime), m_channel(channel), m_collisions(collisions), m_bpsk(bpsk) { 
 	assert(channel.m_numberOfAgents == m_numberOfAgents);
	assert(m_pAgents.size() == m_numberOfAgents);

	m_pAgentPositions = new const vec3*[m_numberOfAgents];

	for (unsigned int i = 0; i < m_numberOfAgents; i++){
		m_pAgentPositions[i] = &(m_pAgents[i]->m_pData->m_pos);
	}  
}

template <class MyData>
SimulationEnvironment<MyData>::~SimulationEnvironment(){
	delete[] m_pAgentPositions;
}

template <class MyData>
void SimulationEnvironment<MyData>::_send(const AgentID sendingAgent, const MyData& data){
}

template <class MyData>
void SimulationEnvironment<MyData>::send(const AgentID sendingAgent){
	_send(sendingAgent, m_pAgents[sendingAgent]->m_pData);
}

template <class MyData>
int SimulationEnvironment<MyData>::process(){
	// Extra random number generator
	std::default_random_engine* pRandomGenerator = m_channel.m_fading.m_pRandomGenerator; 

	//initialize channel
	m_channel.init();
	
	//preprocessing phase
	for (AgentID i = 0; i< m_numberOfAgents; i++){
		m_pAgents.at(i)->pre_process();
	}

	//Sending/receiving phase:
	// 1) Sending
	for (AgentID i = 0; i< m_numberOfAgents; i++){
		if (! m_pAgents.at(i)->get_send_flag()){
			continue;
		}
		SlotNumber slot = m_channel.sample_slot();
		MyData* pData = m_pAgents.at(i)->get_data();
		m_channel.send(i, slot, *pData);
	}

	// 2) receiving
	for (SlotNumber k = 0; k < m_channel.m_numberOfSlots; k++){
		m_channel.reset_currently_receiving_List();
		auto sendingAgentsInSlot_k = m_channel.get_sending_agents_by_slot(k);

		if (sendingAgentsInSlot_k.size() == 0){
			continue;
		} 

		m_channel.m_fading.sample_by_slot(sendingAgentsInSlot_k, m_pAgentPositions);

		// u ---> v
		const double sinrThreshold = m_channel.m_fading.m_protocol.sinr_threshold();
		const double P_N = m_channel.m_fading.m_protocol.noise();

		unsigned int i = 0;
		for (auto& u : m_channel.m_dataSentBySlot[k]){
			AgentID id_u = std::get<0>(u);
			assert(sendingAgentsInSlot_k[i] == id_u);

			for (AgentID id_v = 0; id_v < m_numberOfAgents; id_v++){
				if (id_u == id_v)
					{continue;} 

				double interference = 0;
				bool agent_vSendingInSlot_k = false;
				for (AgentID id_l : sendingAgentsInSlot_k){
					if (id_l == id_u)
						continue;
					if (id_l == id_v){
                        agent_vSendingInSlot_k  = true;
                        break;
					}
					interference += m_channel.m_fading.m_interferenceInCurrentSlot[id_l][id_v];
				}

				if (agent_vSendingInSlot_k){
					continue;
				}

				double& P_I = interference;
				const double  P_S = m_channel.m_fading.m_interferenceInCurrentSlot[id_u][id_v];

				// SINR model
				const double sinr = P_S/(P_I + P_N);
				const double packetSize = m_channel.m_fading.m_protocol.m_bitrate * m_samplingTime/static_cast<double>(m_channel.m_numberOfSlots);
                
                // Determine transmission success
                bool received;
                if (m_bpsk) {
                    received = bpsk_sinr_success_function(packetSize, m_channel.m_fading.m_protocol.m_channelBandwidth, m_channel.m_fading.m_protocol.m_bitrate, sinr, sinrThreshold, pRandomGenerator);
                } else {
                    received = default_sinr_success_function(packetSize, m_channel.m_fading.m_protocol.m_channelBandwidth, m_channel.m_fading.m_protocol.m_bitrate, sinr, sinrThreshold, pRandomGenerator);
                }

				if (received){
					if (m_collisions != CollisionBehaviour::ReceiveAll){
						//register all candidates that exceed the threshold
						m_channel.m_currentlyReceivingFromList[id_v].push_back(std::make_tuple(id_u, &(std::get<1>(u))));
					}
					else{
						m_pAgents[id_v]->receive_data(id_u, k, std::get<1>(u));
					} 
				}
			}
			i++;
		} 

		// if requested, ensure that each agent receives at most one message per slot
		for (AgentID id_v = 0; m_collisions != CollisionBehaviour::ReceiveAll && id_v < m_numberOfAgents; id_v++){
			const int chosenPositionReturnValue = m_channel.pick_position_from_currentlyReceivingFromList(id_v);
            const size_t noReceived = m_channel.m_currentlyReceivingFromList[id_v].size();
            
			if (!(chosenPositionReturnValue == -1 || (m_collisions == CollisionBehaviour::DropAll && noReceived >= 2))){
				const unsigned int chosenPosition = static_cast<const unsigned int>(chosenPositionReturnValue);	
				//receving:
				AgentID chosenSender = std::get<0>(m_channel.m_currentlyReceivingFromList[id_v].at(chosenPosition));

				MyData* data  = std::get<1>(m_channel.m_currentlyReceivingFromList[id_v].at(chosenPosition));
				m_pAgents[id_v]->receive_data(chosenSender, k, *data);
			} 	
		}

		//postprocessing phase
		for (unsigned int i = 0; i< m_numberOfAgents; i++){
			m_pAgents.at(i)->post_process();
		} 
	} 
	
	return 0;
}

// Add compatibility check of MyAgent and MyData
template <class MyAgent, class MyData, int maxNumberOfAgents>
class SEMemory{
	public:
		MyAgent m_agentMemory[maxNumberOfAgents];
		MyData m_dataMemory[maxNumberOfAgents];
		std::vector<Agent<MyData>*> m_pAgents;
		const unsigned int m_numberOfAgents;
		NetworkChannel<MyData> m_channel;
 		SimulationEnvironment<Data>* m_simMem;
		double m_beaconFreq;          

		SEMemory(const unsigned int numberOfAgents, SINR::Fading& fading, const unsigned int numberOfSlots, const int slotSeed, const double beaconFreq) : m_numberOfAgents(numberOfAgents), m_channel(NetworkChannel<MyData>(fading, numberOfAgents, numberOfSlots, slotSeed)), m_simMem(nullptr), m_beaconFreq(beaconFreq){
				assert(m_numberOfAgents <= maxNumberOfAgents);
				if (m_numberOfAgents > maxNumberOfAgents){
					std::cerr << "numberOfAgents must not exceed maxNumberOfAgents" << std::endl;
					exit(1);
				}
		}; 
                ~SEMemory(){
			if (m_simMem != nullptr){
				delete m_simMem;
			}
		}
                                                                                                                                                                                              

       	SimulationEnvironment<Data>* create(const CollisionBehaviour collisions, const bool bpsk){
            for (unsigned int i = 0; i < m_numberOfAgents; i++){
				m_dataMemory[i].init({0.0, 0.0, 0.0});
				m_agentMemory[i].init(&(m_dataMemory[i]), static_cast<unsigned int>(m_numberOfAgents), m_beaconFreq);
				m_pAgents.push_back(&(m_agentMemory[i]));
			} 
			m_simMem = new SimulationEnvironment<MyData>(m_numberOfAgents, m_pAgents, 0.0, m_channel, collisions, bpsk);
			return m_simMem;
		}
};

template <class TheData>
class NetworkChannel{
	public:
		SINR::Fading& m_fading;
		const unsigned int m_numberOfAgents;
		const unsigned int m_numberOfSlots;
		const unsigned int m_slotSeed;

		typedef std::tuple<AgentID,TheData> t2_uintData;
		typedef std::vector<t2_uintData> DataContainer;

		typedef std::tuple<AgentID,TheData*> t2_uintpData;

		DataContainer* m_dataSentBySlot;
		std::vector<t2_uintpData>* m_currentlyReceivingFromList;

		NetworkChannel(SINR::Fading& fading, const unsigned int numberOfAgents, const unsigned int numberOfSlots, const int slotSeed);
		~NetworkChannel();
		void init();
		void send(const AgentID sendingAgent, const SlotNumber slot, const TheData& data);
		SlotNumber sample_slot();
		std::vector<AgentID> get_sending_agents_by_slot(SlotNumber k);
		void reset_currently_receiving_List();

		int pick_position_from_currentlyReceivingFromList(AgentID receiver);
};

// ################### BEGIN class NetworkChannel ###################
//
// Due to compilation errors regarding the template mechanism it is necessary 
// to put the class code into this header file
//
		
template <class TheData>
NetworkChannel<TheData>::NetworkChannel(SINR::Fading& fading, const unsigned int numberOfAgents, const unsigned int numberOfSlots, const int slotSeed): m_fading(fading),m_numberOfAgents(numberOfAgents), m_numberOfSlots(numberOfSlots), m_slotSeed(slotSeed){
	assert(fading.m_numberOfAgents == m_numberOfAgents);

	m_dataSentBySlot = new DataContainer[m_numberOfSlots];
	srand(m_slotSeed);

	m_currentlyReceivingFromList = new std::vector<t2_uintpData>[m_numberOfAgents];
}

template <class TheData>
NetworkChannel<TheData>::~NetworkChannel(){
	delete[] m_currentlyReceivingFromList;
	delete[] m_dataSentBySlot;
}

template <class TheData>
void NetworkChannel<TheData>::init(){
	for (SlotNumber k = 0; k <  m_numberOfSlots; k++){
		m_dataSentBySlot[k].clear();
	} 
}

template <class TheData>
void NetworkChannel<TheData>::reset_currently_receiving_List(){
	for (AgentID i = 0; i < m_numberOfAgents; i++){
		m_currentlyReceivingFromList[i].clear();
	}
}

template <class TheData>
int NetworkChannel<TheData>::pick_position_from_currentlyReceivingFromList(AgentID receiver){
	size_t len = m_currentlyReceivingFromList[receiver].size();

	if (len >= 1){
		return rand()%len;
	}	
	else if (len == 0){
		return -1;
	} 
} 

template <class TheData>
SlotNumber NetworkChannel<TheData>::sample_slot(){
	return rand()%m_numberOfSlots;
} 

template <class TheData>
void NetworkChannel<TheData>::send(const AgentID sendingAgent, const SlotNumber slot, const TheData& data){
	assert(sendingAgent <= m_numberOfAgents);
	assert(slot <= m_numberOfSlots);

	m_dataSentBySlot[slot].push_back(std::make_tuple(sendingAgent, TheData(data)));
	
}

template <class TheData>
std::vector<AgentID> NetworkChannel<TheData>::get_sending_agents_by_slot(SlotNumber k){
	std::vector<AgentID> ret;

	for (auto v : m_dataSentBySlot[k]){
		ret.push_back(std::get<0>(v));
	}
	return ret;

} 
// ################### END class NetworkChannel ###################

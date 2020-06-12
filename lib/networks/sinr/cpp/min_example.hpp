#pragma once
#include <iostream>
#include <stdio.h>
#include <tuple>
#include <random>
#include <assert.h>
#include "fading.h"

// uncomment here
//#include "mex.hpp"
//#include "StructArray.hpp"
//
//using matlab::mex::ArgumentList;

// remove dummy structArray 
namespace matlab{
	namespace data{
		typedef std::vector<double> StructArray;
	}
}

/* #######################################################
 * #
 * # min_example.hpp
 * # Author: Daniel Schneider [ds] (schneiderd@uni-koblenz.de)
 * # Date: 2020-06-09
 * #
 * # Comments:
 * #
 * #######################################################
 */


// ############## BEGIN: Alias declaration ##############
typedef unsigned int AgentID;
typedef unsigned int SlotNumber;
// ############## END  : Alias declaration ############## 

/*
namespace TLS{
	template <typename T>
	struct Type2Type{
		// Modern C++ Design, A. Alexandrescu, p. 32
		typedef T OriginalType;
	}; 
}
*/

//simple positon data structure
/*
struct vec3{
	double x1;
	double x2;
	double x3;
}; 
*/
using TLS::vec3;

// Example data type consisting of all data the Agents will exchange from within MATLAB.
// MATLAB Agents should thus use this data structure directly. 

class Data{
	public:
		vec3 m_pos;
		//matlab::data::StructArray m_structArray;
		Data(){}; 
		//Data(const vec3 pos, const matlab::data::StructArray& structArray): m_pos(pos), m_structArray(structArray){};
		/*
		void update(const vec3& pos, const matlab::data::StructArray& structArray){
			m_pos = pos;	
			m_structArray = structArray;
		}
		*/
}; 


// Agent Base class
template <class TheData>
class Agent{
	public:
		TLS::Type2Type<Data> dataType;
		typedef TheData DataType;

		int set_network_time(const double t);
		static unsigned int m_IDcounter;
		const int m_ID;
		TheData* m_pData;

		Agent() : m_ID(m_IDcounter){
			m_IDcounter++;
		}
		Agent(TheData* pInitialData): m_ID(m_IDcounter), m_pData(pInitialData){
			m_IDcounter++;
		}
		virtual ~Agent() = 0;

		virtual int init(TheData* pInitialData){
		       	m_pData = pInitialData;

			return 0;
		}
		vec3 get_pos();
		void set_pos(const vec3 pos);

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
	std::cout <<  sender << std::endl;
	return 0;
}

template <class TheData>
int Agent<TheData>::post_process(){
	return 0;
}

//
template <class TheData>
//Must be def. outside the class and will be set to zero (Scott Meyers, effective c++, p.59).
//
unsigned int Agent<TheData>::m_IDcounter; 
// ################### END class Agent ################### 


// Simple Agent
class SomeAgent : public Agent<Data>{
	public:
		unsigned int m_numberOfAgents;
		double m_beaconFreq;
		std::vector<std::tuple<AgentID, SlotNumber> > m_received_from;

		int init(DataType* pInitialData, const unsigned int numberOfAgents, const double beaconFrequency){
 		       	m_pData = pInitialData;
			m_numberOfAgents = numberOfAgents;
			m_beaconFreq = beaconFrequency; 
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

	    SimulationEnvironment(const unsigned int numberOfAgents, std::vector<BaseAgent*>& agents, const double samplingTime, Channel& channel);
	    ~SimulationEnvironment();

	    void send(const AgentID sendingAgent);

	    int process();
	 protected:
	    void _send(const AgentID sendingAgent, const TheData& data);
	    Channel &m_channel;
	    
}; 

template <class MyData>
SimulationEnvironment<MyData>::SimulationEnvironment(const unsigned int numberOfAgents, std::vector<BaseAgent*>& pAgents, const double samplingTime, Channel& channel)  : m_numberOfAgents(numberOfAgents), m_pAgents(pAgents), m_samplingTime(samplingTime), m_channel(channel){ 

}

template <class MyData>
SimulationEnvironment<MyData>::~SimulationEnvironment(){
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
	//some flags. Need to be refactored.
        constexpr bool eachAgentReceivesAtMostOnePacketPerSlot = true;
	constexpr bool useBernoulliModel = false;
	// needed only for bernoulli
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
		SlotNumber slot = m_channel.sample_slot();
		MyData* pData = m_pAgents.at(i)->get_data();
		m_channel.send(i, slot, *pData);
	}

	// 2) receiving
	for (SlotNumber k = 0; k < m_channel.m_numberOfSlots; k++){
		m_channel.reset_currently_receiving_List();
		auto sendingAgentsInSlot_k = m_channel.get_sending_agents_by_slot(k);

		if (sendingAgentsInSlot_k.size() == 0){
			//std::cout << "skipping..." << std::endl;	
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
					//if ((id_l == id_u) || (id_l == id_v))
					if (id_l == id_u)
						{continue;}
					if (id_l == id_v){
						// ToDo: To be refactored. Transform this adhoc-skip-rubbish into a 
						// self-explaining flag argument:
						// on/off: sending nodes can/cannot receive.
						//
						// SKIP=true means that the check if the receiving node is sending
						// is skipped.
						// Thus: 
						// 	SKIP=true 	<==>  "sending nodes *can* receive"
						constexpr bool SKIP = false;
						
						if (!SKIP){
							//regular case
							agent_vSendingInSlot_k  = true;
							break;
						}
						else{
							// DEBUGING: SKIP == true
							continue;
						}
					}
					interference += m_channel.m_fading.m_interferenceInCurrentSlot[id_l][id_v];
				}

				if (agent_vSendingInSlot_k){
					continue;
				}

				double& P_I = interference;
				const double  P_S = m_channel.m_fading.m_interferenceInCurrentSlot[id_u][id_v];


				if (!useBernoulliModel){
					// SINR model
					if (P_S/(P_I + P_N) >= sinrThreshold){
						if (eachAgentReceivesAtMostOnePacketPerSlot){
							//register all candidates that exceed the threshold
							m_channel.m_currentlyReceivingFromList[id_v].push_back(std::make_tuple(id_u, &(std::get<1>(u))));
						}
						else{

							m_pAgents[id_v]->receive_data(id_u, k, std::get<1>(u));
						} 
					} 
					else{
						//std::cout << "no reception" << std::endl;
						//std::cout << m_channel.m_fading.m_interferenceInCurrentSlot[id_u][id_v] - m_channel.m_fading.m_protocol.sinr_threshold() << std::endl;
					}
				}
				else{ //Bernoulli model
					const double p = 0.8;
					//const double prob = p/static_cast<double>(m_channel.m_dataSentBySlot[k].size());
					const double prob = p/(2.0*static_cast<double>(m_channel.m_dataSentBySlot[k].size()));
					std::bernoulli_distribution bernoulli(prob);
					if (static_cast<bool>(bernoulli(*pRandomGenerator))){
						if (eachAgentReceivesAtMostOnePacketPerSlot){
							//register all candidates that exceed the threshold
							m_channel.m_currentlyReceivingFromList[id_v].push_back(std::make_tuple(id_u, &(std::get<1>(u))));
						}
						else{

							m_pAgents[id_v]->receive_data(id_u, k, std::get<1>(u));
						} 
					} 

				}
			}
			i++;
		} 

		// Insure that each agent receives at most one message per slot
		for (AgentID id_v = 0; id_v < m_numberOfAgents; id_v++){
			const int chosenPositionReturnValue = m_channel.pick_position_from_currentlyReceivingFromList(id_v);
			if (chosenPositionReturnValue == -1){
				//std::cout << "sender ============== -1" << std::endl;
				continue;
			}
			else{
				if (eachAgentReceivesAtMostOnePacketPerSlot){
					const unsigned int chosenPosition = static_cast<const unsigned int>(chosenPositionReturnValue);	
					//receving:
					AgentID chosenSender = std::get<0>(m_channel.m_currentlyReceivingFromList[id_v].at(chosenPosition));

					MyData* data  = std::get<1>(m_channel.m_currentlyReceivingFromList[id_v].at(chosenPosition));
					m_pAgents[id_v]->receive_data(chosenSender, k, *data);


				}

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
//template <class MyAgent, class MyData, int n>
class SEMemory{
	public:
		MyAgent m_agentMemory[maxNumberOfAgents];
		MyData m_dataMemory[maxNumberOfAgents];
		//MyAgent m_agentMemory[n];
		//MyData m_dataMemory[n];
		std::vector<Agent<MyData>*> m_pAgents;
		const unsigned int m_numberOfAgents;
		NetworkChannel<MyData> m_channel;
		double m_beaconFreq;          

		//SEMemory(const unsigned int numberOfAgents, const double samplingFrequency) :  m_beaconFreq(samplingFrequency){assert(n <= maxNumberOfAgents)}; 

		SEMemory(const unsigned int numberOfAgents, SINR::Fading& fading, const unsigned int numberOfSlots, const int slotSeed, const double beaconFreq) : m_numberOfAgents(numberOfAgents), m_channel(NetworkChannel<MyData>(fading, numberOfAgents, numberOfSlots, slotSeed)), m_beaconFreq(beaconFreq){
				assert(m_numberOfAgents <= maxNumberOfAgents);
				if (m_numberOfAgents > maxNumberOfAgents){
					std::cerr << "numberOfAgents must not exceed maxNumberOfAgents" << std::endl;
					exit(1);
				}
		}; 
 
                                                                                                                                                                                              

       		SimulationEnvironment<Data> create(){
			//return _create(m_agentMemory[0].dataType);
			return SimulationEnvironment<MyData>(m_numberOfAgents, m_pAgents, 0.0, m_channel);
		}

		/*
		SimulationEnvironment<Data> _create(TLS::Type2Type<Data>&){

			const double factor = 500.0;
			vec3 initPos = {-5.0 * factor, 0.0, 0.0};

			for (unsigned int i = 0; i < m_numberOfAgents; i++){
				initPos.x1 += factor * 1.0;
				initPos.x2 = factor * ( -2.0 + static_cast<double>(i%5));
				m_dataMemory[i].init(initPos, {0.0, 0.0, 0.0});

				m_agentMemory[i].init(&(m_dataMemory[i]), static_cast<unsigned int>(m_numberOfAgents), m_beaconFreq);
				m_pAgents.push_back(&(m_agentMemory[i]));
			}
	
			const double timeInterval = 1e-5;
			const double samplingTime = 5.0 * timeInterval;
		} 
		*/

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


//	protected:

		//intData *currentData;

		//int init_slot();
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

	//std::cout << "cnt = ";
	//std::cout << cnt << std::endl;
	if (len >= 1){
		if (len> 1){
			//CNT++;
			//std::cerr << "len = ";
			//std::cerr << len << std::endl;
		}

		return rand()%len;
	}	
	else if (len == 0){
		return -1;
	} 
	else{
		assert(false);
	} 
} 


template <class TheData>
SlotNumber NetworkChannel<TheData>::sample_slot(){
	return rand()%m_numberOfSlots;

	//SlotNumber chosen = rand()%m_numberOfSlots;
	//std::cerr << chosen << std::endl;
	//return chosen;
	

} 

template <class TheData>
void NetworkChannel<TheData>::send(const AgentID sendingAgent, const SlotNumber slot, const TheData& data){
	assert(sendingAgent <= m_numberOfAgents);
	assert(slot <= m_numberOfSlots);

	//m_dataSentBySlot[slot].push_back(std::make_tuple(sendingAgent, data));
	m_dataSentBySlot[slot].push_back(std::make_tuple(sendingAgent, TheData(data)));
	
}

template <class TheData>
std::vector<AgentID> NetworkChannel<TheData>::get_sending_agents_by_slot(SlotNumber k){
	std::vector<AgentID> ret;

	for (auto v : m_dataSentBySlot[k]){
		//const typename NetworkChannel<TheData>::AgentID sendingAgent = std::get<0>(v);
		ret.push_back(std::get<0>(v));
	}
	return ret;

} 


// ################### END class NetworkChannel ###################
 

/* -------------------------------------------------------------------------------------------------
 * Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved
 * Licensed under the GPLv3. See LICENSE in the project root for license information.
 * Author(s): Daniel Schneider
 *            Christian Hespe
 * -------------------------------------------------------------------------------------------------
 */

#include <memory>

#include "mex.hpp"
#include "mexAdapter.hpp"

#include "min_example.hpp"
#include <time.h>

// BUILD_VERSION gets set via a compiler argument to the Matlab mex function
#ifndef BUILD_VERSION
    #define BUILD_VERSION -1
#endif

using matlab::mex::ArgumentList;

/**
 * Enum for possible calls to this MEX function.
 */
enum MethodCall {
    Invalid,
    Constructor,
    UpdateNetworkAgent,
    UpdateSendFlag,
    Process,
    UpdateMatlabAgent,
    Destructor,
    BuildVersion,
};

/**
 * Enum for wireless protocols that are defined in this library
 */
enum WirelessProtocolEnum {
    wp_802_11n_mode_1,
    wp_802_11n_mode_2,
    wp_802_11p,
    wp_802_15_4_europe,
    underwater_mako_modem,
    underwater_marlin_modem,
    Unknown,
};

class MexFunction : public matlab::mex::Function {
private:
    static const unsigned MAX_AGENTS = 1000;
    
    // Would both be better if we use std::optional -> requires C++17
    std::unique_ptr<SEMemory<SomeAgent, Data, MAX_AGENTS>> memory;
    SimulationEnvironment<Data>* pSim;
    std::unique_ptr<SINR::WirelessProtocolParameters> wp;
    std::unique_ptr<SINR::Fading::PowerVec> powerVec;
    std::unique_ptr<SINR::NakagamiFading> fading;    

    // Required for printing error messages in Matlab
    std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
    
    // Creates return values
    matlab::data::ArrayFactory factory;
    
    MethodCall parseCall(std::string name) const {
        if (name == "new")                 return MethodCall::Constructor;
        if (name == "updateNetworkAgent")  return MethodCall::UpdateNetworkAgent;
        if (name == "updateSendFlag")  	   return MethodCall::UpdateSendFlag;
        if (name == "process")             return MethodCall::Process;
        if (name == "updateMatlabAgent")   return MethodCall::UpdateMatlabAgent;
        if (name == "delete")              return MethodCall::Destructor;
        if (name == "buildVersion")        return MethodCall::BuildVersion;
        
        return MethodCall::Invalid;
    }
    
    WirelessProtocolEnum parseProtocol(std::string name) const {
        if (name == "wp_802_11n_mode_1")         return WirelessProtocolEnum::wp_802_11n_mode_1;
        if (name == "wp_802_11n_mode_2")         return WirelessProtocolEnum::wp_802_11n_mode_2;
        if (name == "wp_802_11p")                return WirelessProtocolEnum::wp_802_11p;
        if (name == "wp_802_15_4_europe")        return WirelessProtocolEnum::wp_802_15_4_europe;
        if (name == "underwater_mako_modem")     return WirelessProtocolEnum::underwater_mako_modem;
        if (name == "underwater_marlin_modem")   return WirelessProtocolEnum::underwater_marlin_modem;
        
        return WirelessProtocolEnum::Unknown;
    }
    
    /**
     * Print message in the Matlab command line
     */
    void display(std::string message) {
        matlabPtr->feval(u"disp", 0, std::vector<matlab::data::Array>({ factory.createCharArray(message) }));
    }
    
    /**
     * Raise Matlab error. Return values will not be considered
     */
    void error(std::string message) {
        matlabPtr->feval(u"error", 0, std::vector<matlab::data::Array>({ factory.createCharArray(message) }));
    }
    
public:
    MexFunction() : pSim(nullptr){}; 
    
    SimulationEnvironment<Data>* get_sim(){
	    return pSim;
    }
    
    void operator()(ArgumentList outputs, ArgumentList inputs) {
	/******************
	 *   Assumes *inputs* is of the following form:
	 *   	
	 *   	inputs[0]: CharArray/string (the method call)
	 *   	inputs[k], k > 0: see switch statement below
	 *
	 * ***/

        matlab::data::CharArray methodName = inputs[0];
        MethodCall call = parseCall(methodName.toAscii());     

        switch(call) {
            case MethodCall::Constructor:
            {
            /* ***************
             *
             * Assumes input[1] is SinrConfiguration object
             *
             * ****/ 

                matlab::data::Array config(inputs[1]);
                unsigned numberOfAgents    = matlabPtr->getProperty(config, u"agentCount")[0];
                unsigned numberOfSlots     = matlabPtr->getProperty(config, u"slotCount")[0];
                unsigned dataSize          = matlabPtr->getProperty(config, u"packetSize")[0];
                unsigned fadingSeed        = matlabPtr->getProperty(config, u"fadingSeed")[0];
                unsigned slotSeed          = matlabPtr->getProperty(config, u"slotSeed")[0];
                double   cycleTime         = matlabPtr->getProperty(config, u"cycleTime")[0];
                double   power             = matlabPtr->getProperty(config, u"power")[0];
                double   pathLoss          = matlabPtr->getProperty(config, u"pathLoss")[0];
                double   nakagamiParameter = matlabPtr->getProperty(config, u"nakagamiParameter")[0];
                double   temperature       = matlabPtr->getProperty(config, u"temperature")[0];
                bool     dropCollisions    = matlabPtr->getProperty(config, u"dropCollisions")[0];
                
                // Check if the number of agents is in the valid range
                if( numberOfAgents < 1 || MAX_AGENTS < numberOfAgents ) {
                    error("The number of agents must be between 1 and 1000");
                    return;
                }

                matlab::data::EnumArray protocol(matlabPtr->getProperty(config, u"wirelessProtocol"));
                switch(parseProtocol(std::string(protocol[0]))){
                    case WirelessProtocolEnum::wp_802_11n_mode_1:
                        wp = std::unique_ptr<SINR::WirelessProtocolParameters>(new SINR::WirelessProtocol802_11n_mode1(temperature));
                        break;
                    case WirelessProtocolEnum::wp_802_11n_mode_2:
                        wp = std::unique_ptr<SINR::WirelessProtocolParameters>(new SINR::WirelessProtocol802_11n_mode2(temperature));
                        break;
                    case WirelessProtocolEnum::wp_802_11p:
                        wp = std::unique_ptr<SINR::WirelessProtocolParameters>(new SINR::WirelessProtocol802_11p(temperature));
                        break;
                    case WirelessProtocolEnum::wp_802_15_4_europe:
                        wp = std::unique_ptr<SINR::WirelessProtocolParameters>(new SINR::WirelessProtocol802_15_4_europe(temperature));
                        break;
                    case WirelessProtocolEnum::underwater_mako_modem:
                        // Here are acutally further settings possible: shippingParameter, windParameter (currently using default values).
                        wp = std::unique_ptr<SINR::WirelessProtocolParameters>(new SINR::UnderwaterProtocol(SINR::Mako));
                        break;
                    case WirelessProtocolEnum::underwater_marlin_modem:
                        // Here are acutally further settings possible: shippingParameter, windParameter (currently using default values).
                        wp = std::unique_ptr<SINR::WirelessProtocolParameters>(new SINR::UnderwaterProtocol(SINR::Marlin));
                        break;  
                    case WirelessProtocolEnum::Unknown:
                        error("Unknown wireless protocol!");
                        return;
                }
                         
                const double beaconFrequency = 1.0 / cycleTime;
                const double bitrate = beaconFrequency * dataSize * numberOfSlots;
                wp->set_bitrate(bitrate);
                
                powerVec = std::unique_ptr<SINR::Fading::PowerVec>(new SINR::Fading::PowerVec(numberOfAgents, power)); 
                fading   = std::unique_ptr<SINR::NakagamiFading>(new SINR::NakagamiFading(numberOfAgents, fadingSeed, *powerVec, *wp, pathLoss, nakagamiParameter));  	 
                memory   = std::unique_ptr<SEMemory<SomeAgent, Data, MAX_AGENTS>>(new SEMemory<SomeAgent, Data, MAX_AGENTS>(numberOfAgents, *fading, numberOfSlots, slotSeed, beaconFrequency));

                // Create a simulation environment with the correct number of agents
                pSim = memory->create(dropCollisions);

                break;
            }
            case MethodCall::Destructor:
            {
                //ToDo: add remaining destructor calls 
                memory = std::unique_ptr<SEMemory<SomeAgent, Data, MAX_AGENTS>>();

                break;
            }
            case MethodCall::UpdateNetworkAgent:
            {
            /* ***************
             *
             * Assumptions:
             * 	input[1][0] is unsigned int (the agent id)
             * 	input[2]    is a double array of dimension (3,1) (the agent's position)
             *
             * ****/

                if (!memory || pSim == nullptr) {
                    error("You must initialize the network first!");
                    break;
                }
                
                // Agent's ID
                AgentID id = inputs[1][0];

                // Agent's position
                matlab::data::TypedArray<double> position = inputs[2];
                matlab::data::ArrayDimensions dims = position.getDimensions();
                
                if (dims[0] < 1 || dims[0] > 3 || dims[1] != 1) {
                    error("Agent position vector has the wrong dimensions");
                    break;
                }
                
                vec3 setVec = {0.0, 0.0, 0.0};

                switch(dims[0]){
                    case 3:
                        setVec.x3 = position[2];
                    case 2:
                        setVec.x2 = position[1];
                    case 1:
                        setVec.x1 = position[0];
                }

                // update agent's position
                pSim->m_pAgents.at(id-1)->set_pos(setVec);
                
                break;
            }
            case MethodCall::UpdateSendFlag:
            {
            //ToDo: Maybe integrate this call into *UpdateNetworkAgent*

            /* ***************
             *
             * Assumptions:
             * 	input[1][0] is unsigned int (the agent id)
             * 	input[2][0] is a boolean variable (the send flag)
             *
             * ****/

            	if (!memory || pSim == nullptr) {
                    error("You must initialize the network first!");
                    break;
                }
                
                // Agent's ID
                AgentID id = inputs[1][0];

                // Agent's send flag
                bool flag = inputs[2][0];
                
                // update agent's send_flag
                pSim->m_pAgents.at(id-1)->set_send_flag(flag);
                
                break;
            }
            case MethodCall::Process:
            {
                pSim->process();
                break;
            }
            case MethodCall::UpdateMatlabAgent:
            {
            /* ***************
             *
             * Assumptions:
             * 	input[1][0] is unsigned int  (the agent id)
             *
             * ****/ 
                if (!memory || pSim == nullptr) {
                    error("You must initialize the network first!");
                    break;
                }
                
                if (outputs.size() != 1) {
                    error("You must specify exactly one output");
                    break;
                }

                // Agent's ID
                AgentID id = inputs[1][0];               
                std::vector<std::tuple<AgentID, SlotNumber> > receptionList = pSim->m_pAgents.at(id-1)->m_received_from;

                // Maybe necessary to check size?!
                // Create (m,2) Matlab Array, where *m* is the number of received messages. 
                // Each row consists of the sender's id and the slot in which the message was received.
                auto returnVec = factory.createArray<unsigned int>({ receptionList.size(), 2});
		
                int i = 0;
                for (auto &tuple : receptionList){
                    returnVec[i][0] = std::get<0>(tuple) + 1;
                    returnVec[i][1] = std::get<1>(tuple) + 1;

                    i++;
                }
                outputs[0] = returnVec;
                
                break;
            }
            case MethodCall::BuildVersion:
            {
                if (outputs.size() != 1) {
                    error("You must specify exactly one output");
                    break;
                }
                
                outputs[0] = factory.createScalar<int>(BUILD_VERSION);
                
                break;
            }
            case MethodCall::Invalid:
            {
                error("Invalid function call!");
                break;
            } 
        }
    }
};

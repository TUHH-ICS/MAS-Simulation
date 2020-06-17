#include <memory>

#include "mex.hpp"
#include "mexAdapter.hpp"

#include "min_example.hpp"
#include <time.h>

using matlab::mex::ArgumentList;

/**
 * Enum for possible calls to this MEX function.
 */
enum MethodCall {
    Invalid,
    Constructor,
    UpdateNetworkAgent,
    Process,
    UpdateMatlabAgent,
    Destructor,
};

class MexFunction : public matlab::mex::Function {
private:
    static const unsigned MAX_AGENTS = 50;
    
    // Would both be better if we use std::optional -> requires C++17
    std::unique_ptr<SEMemory<SomeAgent, Data, MAX_AGENTS>> memory;
    SimulationEnvironment<Data>* pSim;
    std::unique_ptr<SINR::WirelessProtocol802_11p> wp;
    std::unique_ptr<SINR::Fading::PowerVec> powerVec;
    std::unique_ptr<SINR::NakagamiFading> fading;    

    // Required for printing error messages in Matlab
    std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
    
    // Creates return values
    matlab::data::ArrayFactory factory;
    
    MethodCall parseCall(std::string name) const {
        if (name == "new")                 return MethodCall::Constructor;
        if (name == "updateNetworkAgent")  return MethodCall::UpdateNetworkAgent;
        if (name == "process")             return MethodCall::Process;
        if (name == "updateMatlabAgent")   return MethodCall::UpdateMatlabAgent;
        if (name == "delete")              return MethodCall::Destructor;
        
        return MethodCall::Invalid;
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
             * Assumes input[1][0] is unsigned int
             *
             * ****/ 

                unsigned int numberOfAgents = inputs[1][0];
                
                // Check if the number of agents is in the valid range
                if( numberOfAgents < 1 || MAX_AGENTS < numberOfAgents ) {
                    error("The number of agents must be between 1 and 50");
                    break;
                }

                unsigned int seed = time(NULL);
                unsigned int fadingSeed = seed;
                unsigned int slotSeed = seed;
                const double pathLoss = 2.0;
                const double nakagamiParameter = 2.0;
                constexpr int maxNumberOfAgents = 100;
                const int numberOfSlots = 5;

                const unsigned int dataSize = 400 * 6 * 64; // [bit]
                const double relBitrate = 0.4 * numberOfSlots;
                const double mW = 0.001;
                const double power = 500.0 * mW;
                wp = std::unique_ptr<SINR::WirelessProtocol802_11p>(new SINR::WirelessProtocol802_11p);
                powerVec = std::unique_ptr<SINR::Fading::PowerVec>(new SINR::Fading::PowerVec(numberOfAgents, power)); 
                fading = std::unique_ptr<SINR::NakagamiFading>(new SINR::NakagamiFading(numberOfAgents, fadingSeed, *powerVec, *wp, pathLoss, nakagamiParameter));  	 

                const double bitrate = relBitrate * wp->m_channelCapacity;
                wp->set_bitrate(bitrate);
                const double beaconFrequency = bitrate/(dataSize * numberOfSlots); 

                memory = std::unique_ptr<SEMemory<SomeAgent, Data, MAX_AGENTS>>(new SEMemory<SomeAgent, Data, MAX_AGENTS>(numberOfAgents, *fading, numberOfSlots, slotSeed, beaconFrequency));

                // Create a simulation environment with the correct number of agents
                pSim = memory->create();

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
            case MethodCall::Invalid:
            {
                error("Invalid function call!");
                break;
            } 
        }
    }
};

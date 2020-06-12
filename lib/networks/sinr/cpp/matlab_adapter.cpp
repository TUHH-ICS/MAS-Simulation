#include <memory>

#include "mex.hpp"
#include "mexAdapter.hpp"

#include "min_example.hpp"

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

/**
 * We need to instantiate SEMemory for every possible value of the template parameter n.
 * As this in impossible, this recursively instantiates SEMemory up to a fixed integer.
 * In this way, we can chose the right instance dynamically at runtime, without
 * having to manually define each in a switch/case or if/else statement.
 *
 * See https://stackoverflow.com/a/14417730
 */
/*
template<unsigned n>
struct switcher {
    static std::unique_ptr<MemoryBase> instance(unsigned v) {
        if(v == n){
            // Maybe replace by std::make_unique -> requires C++14
            std::unique_ptr<MemoryBase> ptr(new SEMemory<SomeAgent, Data, n>(0.1));
            return ptr;
        }
        else {
            return switcher<n-1>::instance(v);
        }
    }
};
*/

/**
 * Terminate the above recursive instantiation by specialization of n = 0.
 * Without this, the compiler violates the maximum template recursion depth.
 */
/*
template<>
struct switcher<0> {
    static std::unique_ptr<MemoryBase> instance(unsigned v){
        // This is very unsafe, we need to make sure this is never called.
        // This would not be the case with std::optional -> requires C++17
        return NULL;
    }
};
*/


class MexFunction : public matlab::mex::Function {
private:
    static const unsigned MAX_AGENTS = 50;
    
    // Would both be better if we use std::optional -> requires C++17
    std::unique_ptr<SEMemory<SomeAgent, Data, MAX_AGENTS> > memory;
    std::unique_ptr<SimulationEnvironment<Data> > sim;
    
    // Required for printing error messages in Matlab
    std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
    
    // Creates return values
    matlab::data::ArrayFactory factory;
    
    MethodCall parseCall(std::string name) const {
        if (name == "new")     return MethodCall::Constructor;
        if (name == "updateNetworkAgent")  return MethodCall::UpdateNetworkAgent;
        if (name == "process") return MethodCall::Process;
        if (name == "updateMatlabAgent")   return MethodCall::UpdateMatlabAgent;
        if (name == "delete")  return MethodCall::Destructor;
        
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
    /**
     * Largest number of agents that can be simulated
     */
    
    void operator()(ArgumentList outputs, ArgumentList inputs) {
	/******************
	 *   Assumes *inputs* is of the following form:
	 *   	
	 *   	inputs[0]: CharArray/string (the method call)
	 *   	inputs[k], k > 0: see switch statement below
	 *
	 *
	 * ***/

        matlab::data::CharArray methodName = inputs[0];
        MethodCall call = parseCall(methodName.toAscii());     

        switch(call) {
            case MethodCall::Constructor:
            {
                 /* ***************
		 *
		 * Assumes input[1][0] is double 
		 *
		 * ****/ 

                double num = inputs[1][0];
                
                // Check if the number of agents is in the valid range
                if( num < 1 || MAX_AGENTS < num ) {
                    error("The number of agents must be between 1 and 50");
                    break;
                }
                
                // Instantiating switcher with MAX_AGENTS allows to dynamically create
                // networks of up to MAX_AGENTS agents
                //memory = switcher<MAX_AGENTS>::instance(num);

    		memory = std::unique_ptr<SEMemory>(new SEMemory<SomeAgent, Data, MAX_AGENTS>(num, 0.1));
                
                // Create a simulation environment with the correct number of agents
                SimulationEnvironment<Data> env = memory->create();
                sim = std::unique_ptr<SimulationEnvironment<Data>>(new SimulationEnvironment<Data>(env));
                
                // Return the number of agents to matlab
                if(outputs.size() == 1) {
                    outputs[0] = factory.createScalar(sim->m_numberOfAgents);
                }
                
                break;
            }
            case MethodCall::Destructor:
            {
                memory = std::unique_ptr<MemoryBase>();
                sim = std::unique_ptr<SimulationEnvironment<Data>>();

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

                if (!memory || !sim) {
                    error("You must initialize the network first!");
                    break;
                }
                
		// Agent's ID
                AgentID id = inputs[1][0];

		// Agent's position
                matlab::data::TypedArray<double> position = inputs[2];
                matlab::data::ArrayDimensions dims = position.getDimensions();
                
                if (dims[0] != 3 || dims[1] != 1) {
                    error("Agent position vector has the wrong dimensions");
                    break;
                }
                
		// update agent's position
                sim->m_pAgents[id]->set_pos({ position[0], position[1], position[2] });
		
		/* No longer needed
		// update agent's custum data (matlab StructArray)
                sim->m_pAgents[id].m_pData->m_structArray = inputs[3];
		*/
                
                break;
            }
            case MethodCall::Process:
	    {
		sim->process();
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
                if (!memory || !sim) {
                    error("You must initialize the network first!");
                    break;
                }
                
                if (outputs.size() != 1) {
                    error("You must specify exactly one output");
                    break;
                }
		//
                // Agent's ID
                AgentID id = inputs[1][0]; 

		std::vector<std::tuple<AgentID, SlotNumber> > receptionList = sim->m_pAgents[id]->m_received_from;

		// flatten tuple vector to 1d vector in column-major order
		std::vector<unsigned int> mergedVec, slotVec;
 		for (auto &tuple : receptionList){
			unsigned int sender = std::get<0>(tuple);
			mergedVec.push_back(sender);
			unsigned int slot = std::get<1>(tuple);
			slotVec.push_back(slot);
		} 
 		mergedVec.insert( mergedVec.end(), slotVec.begin(), slotVec.end() );

		// Maybe necessary to check size?!
		// Create (m,2) Matlab Array, where *m* is the number of received messages. 
		// Each row consists of the sender's id and the slot in which the message was received.
                outputs[0] = factory.createArray<unsigned int>({ receptionList.size(), 2}, mergedVec.begin(), mergedVec.end());

                break;
            }
        }
    }
};

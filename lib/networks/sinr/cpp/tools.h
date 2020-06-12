#pragma once
#include <vector>
#include <cmath>
#include <assert.h>
#include <tuple>

 /* *****************************************************
 *  tools.h
 *  Author: Daniel Schneider [ds] (schneiderd@uni-koblenz.de)
 *
 *  Comments:
 *
 *  *****************************************************
*/

namespace TLS{
// Modern C++ Design, A. Alexandrescu, p. 25
template<bool> struct CompileTimeError;
template<> struct CompileTimeError<true> {}; 

#define STATIC_CHECK(expr) (TLS::CompileTimeError<(expr) != 0>())
 



template <int v>
struct Int2Type{
	// Modern C++ Design, A. Alexandrescu, p. 29
	enum { value = v };
};

template <typename T>
struct Type2Type{
	// Modern C++ Design, A. Alexandrescu, p. 32
	typedef T OriginalType;
};
struct vec3{
	double x1;
	double x2;
	double x3;
};
void set_vec3(vec3& target, const vec3& source);


double vec3_squared_dist(const vec3& a, const vec3& b);

double vec3_dist(const vec3& a, const vec3& b);

vec3 vec3_sum(const vec3& a, const vec3& b);
vec3 vec3_difference(const vec3& a, const vec3& b);
vec3 vec3_scal_mult(const double& lambda, const vec3& a);

double vec3_len(const vec3& a);


bool my_fmod(double x, double y);



template <typename T>
class VarSum{
	public:
 		int* m_lowerBounds;
		int* m_upperBounds;
		unsigned int m_len;  	

		VarSum();
		virtual ~VarSum();

		//void generate_Int2Types(const unsigned int maxInt);

                void set_summation_bounds(const std::vector<int>& lowerBounds, const std::vector<int>& upperBounds);

		template <int v>
		long double var_sum(Int2Type<v> summandVariant, T& data);

		template <int v>
		long double var_sum_F2(Int2Type<v> summandVariant, const unsigned int& numberOfVars, T& data);

		//template <int v>
		//double summand(Int2Type<v>& summandVariant, const unsigned int&, const int*){return 1.0;};

       		//double summand(Int2Type<0>&, const unsigned int&, const int*){return 1.0;};
       		//double summand(Int2Type<0>&, const unsigned int&, const int*, T& data){return 1.0;};

		/*
       		virtual double summand(Int2Type<1>&, const unsigned int&, const int*){assert(false); return 1.0;};
                virtual double summand(Int2Type<2>&, const unsigned int&, const int*){assert(false); return 1.0;};
                virtual double summand(Int2Type<11>&, const unsigned int&, const int*){assert(false); return 1.0;};
                virtual double summand(Int2Type<12>&, const unsigned int&, const int*){assert(false); return 1.0;};
		*/

       		virtual double summand(Int2Type<0>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;};
       		virtual double summand(Int2Type<1>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;};
       		virtual double summand(Int2Type<2>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;}; 
       		virtual double summand(Int2Type<3>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;}; 
       		virtual double summand(Int2Type<4>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;}; 
       		virtual double summand(Int2Type<5>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;}; 
       		virtual double summand(Int2Type<6>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;}; 
       		virtual double summand(Int2Type<7>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;}; 
       		virtual double summand(Int2Type<8>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;}; 
       		virtual double summand(Int2Type<9>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;}; 
       		virtual double summand(Int2Type<10>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;}; 
       		virtual double summand(Int2Type<11>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;}; 
       		virtual double summand(Int2Type<12>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;};  
       		virtual double summand(Int2Type<13>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;};  
       		virtual double summand(Int2Type<14>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;};  
       		virtual double summand(Int2Type<15>&, const unsigned int&, const int*, T& data){assert(false); return 1.0;};  

		/*
       		virtual double summand(Int2Type<2>&, const unsigned int&, const int*){assert(false); return 1.0;};
       		virtual double summand(Int2Type<3>&, const unsigned int&, const int*){assert(false); return 1.0;};
       		virtual double summand(Int2Type<4>&, const unsigned int&, const int*){assert(false); return 1.0;};
       		virtual double summand(Int2Type<5>&, const unsigned int&, const int*){assert(false); return 1.0;};
       		virtual double summand(Int2Type<6>&, const unsigned int&, const int*){assert(false); return 1.0;};
       		virtual double summand(Int2Type<7>&, const unsigned int&, const int*){assert(false); return 1.0;};
       		virtual double summand(Int2Type<8>&, const unsigned int&, const int*){assert(false); return 1.0;};
       		virtual double summand(Int2Type<9>&, const unsigned int&, const int*){assert(false); return 1.0;};
       		virtual double summand(Int2Type<10>&, const unsigned int&, const int*){assert(false); return 1.0;};
       		virtual double summand(Int2Type<11>&, const unsigned int&, const int*){assert(false); return 1.0;};
       		virtual double summand(Int2Type<12>&, const unsigned int&, const int*){assert(false); return 1.0;};
       		virtual double summand(Int2Type<13>&, const unsigned int&, const int*){assert(false); return 1.0;};
       		virtual double summand(Int2Type<14>&, const unsigned int&, const int*){assert(false); return 1.0;};
       		virtual double summand(Int2Type<15>&, const unsigned int&, const int*){assert(false); return 1.0;};
		*/


		//template <int v>
		//double summand_F2(Int2Type<v>& summandVariant, const unsigned int& argc, const short* args){return 1.0;};
		
		double summand_F2(Int2Type<0>& summandVariant, const unsigned int& argc, const short* args){return 1.0;};
		/*
		virtual double summand_F2(Int2Type<1>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<2>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<11>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<12>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		*/
		virtual double summand_F2(Int2Type<1>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<2>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<3>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<4>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<5>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<6>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<7>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<8>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<9>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<10>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<11>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<12>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;}; 	
		virtual double summand_F2(Int2Type<13>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;}; 	
		virtual double summand_F2(Int2Type<14>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;}; 	
		virtual double summand_F2(Int2Type<15>&, const unsigned int& argc, const short* args, T& data){assert(false); return 1.0;}; 	
       
	
		/*
		virtual double summand_F2(Int2Type<3>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<4>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<5>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<6>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<7>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<8>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<9>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<10>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<13>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<14>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		virtual double summand_F2(Int2Type<15>&, const unsigned int& argc, const short* args){assert(false); return 1.0;};
		*/


	private:
		int* m_args;
		T* m_pTmpData;

		template <int v>
		long double _var_sum(Int2Type<v>& summandVariant, const unsigned int& n);
		void clean_up();


}; 

template <typename T>
VarSum<T>::VarSum() : m_lowerBounds(nullptr), m_upperBounds(nullptr),  m_len(0), m_args(nullptr), m_pTmpData(nullptr){
	// ************************************ class VarSum ***********************************************
	// * ToDo: in var_sum the variable argument changed from double* to int*. We have to correct this  *
	// *       here as well.                                                                           *
	// *                                                                                               *
	// * A class implementing nested sums. Also see the lightweight function implementation *var_sum*  *
	// *                                                                                               *
	// * Usage: Define a function *summand* which is called during summmation as follows:              *
	// * Inherit from this class and overload the function *f*.                                        *
	// * The first argument of *summand* is the number of summation variables.                         *
	// * The second one is a pointer to the array of summation variables.                              *
	// *                                                                                               *
	// * EXAMPLE:                                                                                      *
	// *    class ExemplaryUseMyVarSum : public VarSum<DataType> {                                               *
	// *		public:                                                                            *
	// *			using VarSum::VarSum;                                                      *
	// *			double summand(const unsigned int& argc, const int* vars, DataType& data){                           *
	// *			for (unsigned int i = 0; i < argc; i++){                                   *
	// *				std::cout << vars[i] << std::endl;                                 *
        // *	         	 }                                                                         *
	// *                                                                                               *
	// *                    return 0.0;};                                                              *
	// *                                                                                               *
	// *	};                                                                                         *
	// * USAGE:                                                                                        *
	// *	std::vector<int> lb = {1,  0};                                                             *
	// * 	std::vector<int> ub = {10, 2};								   *
	// *    ExemplaryUseMyVarSum vs(lb, ub);                                                           *     
	// *    std::cout << vs.sum() << std::endl;                                                        *
	// *                                                                                               *
	// *                                                                                               *
	// *************************************************************************************************


	/*
	 * // To be removed
	m_args = new int[m_len];
	m_lowerBounds = new int[m_len];
	m_upperBounds = new int[m_len];

 	for (unsigned int i = 0; i< m_len; i++){
		m_lowerBounds[i] = 0;
		m_upperBounds[i] = 0;
	}  
	*/

}

template <typename T>
template <int v>
long double VarSum<T>::_var_sum(Int2Type<v>& summandVariant, const unsigned int& n){
	long double sum = 0.0;
	if (n > 1){
		for (int c = m_lowerBounds[m_len - n]; c < m_upperBounds[m_len - n]; c++){
			m_args[m_len - n] = c;
			sum += _var_sum(summandVariant, n-1);
		} 
	}
	else if (n == 1){
		for (int c = m_lowerBounds[m_len - n]; c < m_upperBounds[m_len - n]; c++){
			m_args[m_len-1] = c;
			sum += summand(summandVariant, m_len, m_args, *m_pTmpData);
		}
	}
	else{
		// maybe summantion bounds not set?
		//std::cerr << "ERROR: maybe summantion bounds not set?" << std::endl;
		assert(false);
	}

	return sum;
}                                

template <typename T>
template <int v>
long double VarSum<T>::var_sum(Int2Type<v> summandVariant, T& data){
	m_pTmpData = &data;
	//return static_cast<double>(_var_sum(summandVariant, m_len));
	return (_var_sum(summandVariant, m_len));
} 


template <typename T>
template <int v>
long double VarSum<T>::var_sum_F2(Int2Type<v> summandVariant, const unsigned int& numberOfVars, T& data){
	// A special implementation for sums having bounds 0 and 1 only (i.e. vars are 
	// elements of F_2 ~ Z/2Z).
	//
 	// The variable *c* is treated as *m_numberOfVars-1* dimensional vector over F_2 ~ Z/2Z.
	// The nu's element can be accessed by the bitshift expression  below. 

	short* args = new short[numberOfVars];
	long double sum = 0.0;
	for (unsigned int c = 0; c < static_cast<unsigned int>(std::pow(2.0, numberOfVars)); c++){
			
		// Build the vector of 0's and 1's. 
		for (unsigned int j = 0; j < numberOfVars; j++){
	       		args[j] =  (c >> j ) & 1;
		} 

		// call summand
		sum += summand_F2(summandVariant, numberOfVars, args, data);


	}

	delete[] args;

	return sum;
}                                        
 


 
template <typename T>
void VarSum<T>::clean_up(){
	delete[] m_upperBounds;
	delete[] m_lowerBounds;
	delete[] m_args; 
}

template <typename T>
VarSum<T>::~VarSum(){
	clean_up();
} 


template <typename T>
void VarSum<T>::set_summation_bounds(const std::vector<int>& lowerBounds, const std::vector<int>& upperBounds){
	const unsigned int newLen = lowerBounds.size();
	assert(newLen == upperBounds.size());
	assert(newLen >0);

	if (newLen != m_len){
		if (m_len >0){
                       clean_up(); 
		}
 		m_args = new int[newLen];
		m_lowerBounds = new int[newLen];
		m_upperBounds = new int[newLen]; 

		m_len = newLen;
	}

 	for (unsigned int i = 0; i< m_len; i++){
		m_lowerBounds[i] = lowerBounds.at(i);
		m_upperBounds[i] = upperBounds.at(i);
	} 
}
 
class ExemplaryUseMyVarSum : public VarSum<void*> {
	public:
		// init base
		using VarSum<void*>::VarSum;
		// use integers > 0 as template arguments, 0 is reserved for the 1.0-summand of the base class.
		typedef TLS::Int2Type<1> MySpecialSummandDescription;

		double summand(MySpecialSummandDescription& summandVariant, const unsigned int& argc, const int* args, void*){return 1.0;};

}; 
  
int faculty(const unsigned int n);


struct HelperTempData{
	unsigned int t;
	const std::vector<int>& x;
	const unsigned int& id_i;
	const unsigned int& id_j;
	//const unsigned int& id_k;
	const unsigned int& id_l;
	const std::vector<double>& muVec;
	const unsigned int& numberOfAgents;
	const unsigned int& nakagamiParam;
	const unsigned int& mu_ij;
};

class NakagamiFadingHelper : protected TLS::VarSum<HelperTempData>{
 	public:
		typedef TLS::Int2Type<0> GammaPhi;
		typedef std::vector<double> PowerVec;

		const double &m_numberOfAgents;
		const PowerVec& m_powerVec;
		const double &m_theta;
		const double &m_noise;
		const unsigned int &m_nakagamiParam;

		NakagamiFadingHelper(const unsigned int numberOfAgents, const PowerVec& powerVec, const double& theta, const double& noise, const unsigned int nakagamiParam) : VarSum(), m_numberOfAgents(numberOfAgents), m_powerVec(powerVec), m_theta(theta), m_noise(noise), m_nakagamiParam(nakagamiParam){
		}; 

		double gamma(const std::vector<int>& x, const unsigned int& i, const unsigned int& j, const unsigned int& k, const unsigned int& l, const std::vector<double>& muVec, const double mu_ij);

		double summand(GammaPhi&, const unsigned int& argc, const int* vars, HelperTempData& htd);
		
};


/*
template <typename T>
std::tuple<T, T, T, T> sort_four(const T& a,const T& b,const T& c,const T& d);

template <typename T>
T my_build_reduced_index(const T& low, const T& mid1, const T& mid2, const T& high, const T& theInd); 

*/
 
template <typename T>
std::tuple<T, T, T, T> sort_four(const T& a,const T& b,const T& c,const T& d){
    T low1, low2, high1, high2, lowest, middle1, middle2, highest;
    if (a < b){
        low1 = a;
        high1 = b;
    }
    else {
        low1 = b;
        high1 = a;
    }
     
    if (c < d){
        low2 = c;
        high2 = d;
    }
    else{
        low2 = d;
        high2 = c;
    }

    if (low1 < low2){
        lowest = low1;
        middle1 = low2;
    }
    else{
        lowest = low2;
        middle1 = low1;
    }

    if (high1 > high2){
        highest = high1;
        middle2 = high2;
    }
    else{
        highest = high2;
        middle2 = high1;
    }

    if (middle1 < middle2){
        return std::make_tuple(lowest, middle1, middle2, highest);
    }
    else{
        return std::make_tuple(lowest, middle2, middle1, highest);
    }
}
  
template <typename T>
T my_build_reduced_index(const T& low, const T& mid1, const T& mid2, const T& high, const T& theInd){
	T pos;
	if (theInd < mid1){
		if (theInd < low){
			pos = 0;
		}
		else{
			pos = 1;
		}
	}
	else{
		if (theInd < mid2){
			pos = 2;
		}
		else if (theInd < high){
			pos = 3;
		}
		else{
			pos = 4;
		}
	}
	return theInd - pos;
}
     


}//end namespace TLS


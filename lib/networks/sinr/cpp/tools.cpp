#include "tools.h"
#include <iostream>
#include <assert.h>

 /* *****************************************************
 *  tools.cpp
 *  Author: Daniel Schneider [ds] (schneiderd@uni-koblenz.de)
 *  Date: 2020-01-20
 *
 *  Comments:
 *
 *  *****************************************************
*/
 
namespace TLS{

double vec3_squared_dist(const vec3& a, const vec3& b){
	return (a.x1-b.x1)*(a.x1-b.x1) + (a.x2-b.x2)*(a.x2-b.x2) + (a.x3-b.x3)*(a.x3-b.x3);
}

double vec3_dist(const vec3& a, const vec3& b){
	return std::sqrt(vec3_squared_dist(a,b));
}

double vec3_len(const vec3& a){
	return vec3_dist(a, {0.0, 0.0, 0.0});	
}


bool my_fmod(double x, double y){
	const double a = x/y;
	const double b = static_cast<double>(static_cast<int>(x/y));

	return (std::abs(a-b) < 1e-14);
}

void set_vec3(vec3& target, const vec3& source){
	target.x1 = source.x1;
	target.x2 = source.x2;
	target.x3 = source.x3; 
} 

vec3 vec3_sum(const vec3& a, const vec3& b){
	return {a.x1 + b.x1, a.x2 + b.x2, a.x3 + b.x3};
}

vec3 vec3_difference(const vec3& a, const vec3& b){
	return {a.x1 - b.x1, a.x2 - b.x2, a.x3 - b.x3};
} 
vec3 vec3_scal_mult(const double& lambda, const vec3& a){
	return {lambda * a.x1, lambda * a.x2, lambda * a.x3};
} 
int faculty(const unsigned int n){
	int res = 1.0;
	for (unsigned int i = 1; i<= n; i++){
		res *= i;
	}

	return res;
} 
template <const unsigned int m>
long double _var_sum(double (*f)(const unsigned int&, int*), const unsigned int& n, const unsigned int& N, int *args){
	// do not use directly. Use the wrapper *var_sum*
	long double sum = 0.0;
	if (n > 1){
		for (unsigned int c = 0; c < N; c++){
			args[m-n] = c;
			sum += _var_sum<m>(f, n-1, N, args);
		} 
	}
	else if (n == 1){
		for (unsigned int c = 0; c < N; c++){
			args[m-1] = c;
			sum = f(m, args);	
		}
	}
	else{
		assert(false);
	}
	return sum;
}


template <const unsigned int n>
double var_sum(double (*f)(const unsigned int&, int*), const unsigned int& N){
	// Implements the sum
	//
	//     SUM_{c_1, ..., c_n = 0}^{N} the_summand(c_1, ..., c_n).
	//
	// Also see the more detailed class implementation *VarSum*.
	//
	// The function 
	double args[n];
	return static_cast<double>(_var_sum<n>(f, n, N, args));
}

//VarSum::VarSum(const std::vector<int>& lowerBounds, const std::vector<int>& upperBounds) : m_lowerBounds(nullptr), m_upperBounds(nullptr),  m_len(0), m_args(nullptr){



}//end namespace







// for testing

/*
int main(){
 	class MySum : public TLS::VarSum{
		using TLS::VarSum::VarSum;

		//double f(const unsigned int& argc, const double* args){
			//return static_cast<double>(args[0]+ args[1]);
		//}
		double summand(TLS::Int2Type<0>& summandVariant, const unsigned int&, const int*){return 2.0;};
	
	}; 	 
	//MySum vs;
	TLS::VarSum vs;

	//vs.set_summation_bounds(std::vector<int>{0,0}, std::vector<int>{2,2});
	
	std::cout << vs.var_sum_F2(TLS::Int2Type<0>(), 11) << std::endl;

	//std::cout << vs.var_sum(TLS::Int2Type<0>()) << std::endl;
		

}; 
*/


/*
int main(){
	class MySum : public TLS::VarSum{
		using TLS::VarSum::VarSum;

		double f(const unsigned int& argc, const double* args){
			return static_cast<double>(args[0]+ args[1]);
		}
	
	}; 	
	//std::vector<int> lb = {-1, -1, 0};
	//std::vector<int> ub(3, 1);

	//std::vector<int> lb = {1,  0}; 
	//std::vector<int> ub = {10, 2};

	//TLS::VarSum vs(lb, ub);
	//TLS::ExemplaryUseMyVarSum vs(lb, ub);
	//MySum vs(lb, ub);
	//std::cout << vs.sum() << std::endl;
	//
	//
	//const double x = 5.0;


	std::cout << var_sum(my_summand, 2, 3 << std::endl;

	return 0;
} 
*/



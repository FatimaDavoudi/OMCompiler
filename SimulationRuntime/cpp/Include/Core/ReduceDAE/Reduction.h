#pragma once
#include <Core/ReduceDAE/IReduceDAESettings.h>
#include <Core/ReduceDAE/Simulation.h>
//#include <Core/ReduceDAE/IReduceDAE.h>


class Reduction
{
public:
	Reduction(IMixedSystem* system,Simulation& simulation,IReduceDAESettings* settings);
	~Reduction(void);
	//std::vector<unsigned int> DoReduction(label_list_type& labels,ublas::matrix<double>& Ro);
	//methods
	//std::vector<unsigned int> cancelTerms(label_list_type& labels,ublas::matrix<double>& Ro);
	//std::vector<unsigned int> linearizeTerms(label_list_type& labels,ublas::matrix<double>& Ro);
	//std::vector<unsigned int> substituteTerms(label_list_type& labels,ublas::matrix<double>& Ro);
	IReduceDAESettings* _settings;
	ublas::vector<double> getError(ublas::matrix<double>& R,ublas::matrix<double>& R2,vector<string> output_names);
	bool isLess(ublas::vector<double>& v1,ublas::vector<double>& v2,vector<int> indexes,vector<string> output_names);
private:
	IMixedSystem*  _system;
	Simulation& _simulation;
};

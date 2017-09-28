#pragma once
//#include "../../System/Interfaces/IDAESystem.h"
#include <Core/ReduceDAE/IReduceDAESettings.h>

class Ranking
{
public:
	Ranking(IMixedSystem* system,IReduceDAESettings* settings);
	~Ranking(void);
	virtual label_list_type DoRanking(ublas::matrix<double>& R,ublas::matrix<double>& dR,ublas::matrix<double>& Re,vector<double>& time_values);
	label_list_type residuenRanking(ublas::matrix<double>& R,ublas::matrix<double>& dR,ublas::matrix<double>& Re,vector<double>& time_values);
private:
	//methods:
	IReduceDAESettings* _settings;
    IMixedSystem*  _system;
	double	*_zeroVal;
	double	*_zeroValOld;



};
/*
Helper class to select the index of the label tuple  and return it
*/
class Li
{
public:
  typedef  std::tuple_element<0,label_type>::type result_type;
  result_type operator()(const label_type& u) const
  {
      return get<0>(u);
  }
};


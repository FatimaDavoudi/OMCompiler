#include <Core/Modelica.h>
#include "../../Include/Core/ReduceDAE/Ranking.h"
#include "../../Include/Core/ReduceDAE/IReduceDAESettings.h"
#include "../../Include/Core/ReduceDAE/IReduceDAE.h"
#include <boost/foreach.hpp>
#include <ctime>
Ranking::Ranking(IMixedSystem* system,IReduceDAESettings* settings)
	:_system(system)
	,_settings(settings)
{

}

Ranking::~Ranking(void)
{

}

//choose a ranking method
label_list_type Ranking::DoRanking(ublas::matrix<double>& R,ublas::matrix<double>& dR,ublas::matrix<double>& Re,vector<double>& time_values)
{
	if(_settings->getRankingMethod()==IReduceDAESettings::RESIDUEN)
	{

		return residuenRanking(R,dR,Re,time_values);
	}
	else
		throw std::runtime_error("Ranking method not supported yet");
}

//residue ranking
label_list_type Ranking::residuenRanking(ublas::matrix<double>& R,ublas::matrix<double>& dR,ublas::matrix<double>& Re,vector<double>& time_values)
{
	cout<<"Start residue ranking:" << std::endl;
	//cast modelica system to reduce dae object
	IReduceDAE* reduce_dae = dynamic_cast<IReduceDAE*>(_system);
	//cast modelica system to IContinuous object
	IContinuous* continous_system = dynamic_cast<IContinuous*>(_system);
	IEvent* event_system = dynamic_cast<IEvent*>(_system);
	ITime* _timeevent_system = dynamic_cast<ITime*>(_system);
	//cast modelica system to initial system object
	ISystemInitialization* _init_system =  dynamic_cast<ISystemInitialization*>(_system);
	//IHistory* history = reduce_dae->getHistory();
	ublas::matrix<double> Ro;
	//get the number of residues
	//int dimSys = _system->getDimRes();
    int dimSys = 0;

	//reference solution for variables  in  column j in R matrix
	ublas::vector<double> r_j;
	//reference solution for derivatives  in  column j in dR matrix
	ublas::vector<double> dr_j;
	//reference solution for residues  in  column j in Re matrix
	ublas::vector<double> re_j;
	//vector for current error
	ublas::vector<double> e(time_values.size());
	//index for current reduction
	unsigned int k=0;
	//help array for residues
	double *ydHelp;

    double norm_2=0;
	if(reduce_dae)
	{
		//get label variables
		label_list_type $labels = reduce_dae->getLabels();
		//number of labels
		int kDim = $labels.size();
		//vector for ranks
		vector<double> rank_vector(kDim);
//cout << "size labels in ranking:" << kDim << std::endl;
		label_type  $label;
		double time;
		//loop over labels
    BOOST_FOREACH($label, $labels)
		{

			//index for current time entry
			int j=0;


			try
			{

				_timeevent_system->setTime(0);

                _init_system->initialize();

                _init_system->setInitial(true);

				int dim = event_system->getDimZeroFunc();
				bool* conditions0 = new bool[dim];
				bool* conditions1 = new bool[dim];
				//bool restart=true;
				/*int iter = 0;
				while(restart &&!(iter++>10))
				{

					continous_system->evaluateAll(IContinuous::ALL);
					restart = event_system->checkForDiscreteEvents();
				}
				*/
				event_system->getConditions(conditions0);

				event_system->getConditions(conditions1);

				event_system->saveAll();

				_init_system->setInitial(false);

				delete[] conditions0;
				delete[] conditions1;

				 //set label_1 to 0 and label_2 to 1
                    *(get<1>($label))=0;
                    *(get<2>($label))=1;
				 //loop over time entries

                   //double mylabel01 = *(std::get<1>($labels[0]));
                   // double mylabel02 = *(std::get<2>($labels[0]));
                // cout << "Ranking for label: " << get<0>($label) << std::endl;

				BOOST_FOREACH(time, time_values)
				{
                    //cout << "j counter: " << j << std::endl;
					//set time
					//cout << "time " <<time<< std::endl;
					_timeevent_system->setTime(time);
					//continous_system->setTime(time);

					//variables reference solution for time j
					r_j=ublas::column(R,j);
					//cout << "r_j:" << r_j << std::endl;
					//derivative reference solution for time j
					dr_j =ublas::column(dR,j);
					//cout << "dr_j:" << dr_j << std::endl;
					//residues reference solution for time j
					re_j =ublas::column(Re,j);
					//cout << "re_j:" << re_j << std::endl;



					//set reference solution
					reduce_dae->setVariables(r_j,dr_j);
					//update system
					//continous_system->update(IContinuous::RANKING);
					continous_system->evaluateAll(IContinuous::RANKING);
					//get current residues
					//continous_system->giveRHS(ydHelp,IContinuous::ALL_RESIDUES);
                    //IContinuous::UPDATETYPE calltype;
                     //calltype = IContinuous::RANKING;
                     dimSys=continous_system->getDimRHS();

                    ydHelp=new double[dimSys];
					continous_system->getRHS(ydHelp);
                    //_system->getResidual(ydHelp);
					shared_vector_t yd(dimSys,adaptor_t(dimSys,ydHelp));
					//cout << "yd: " << yd << std::endl;
                    //To do after residual only return vlue from algeabric loop equation
					//cout << "yd-dr_j" << yd-dr_j << std::endl;
                    //cout << "yd-dr_j" << yd-dr_j << std::endl;
					//error norm_2 (x)= sqrt (sum |xi|^2 )
					//e(j++) = ublas::norm_2(yd-re_j);
                     norm_2 = ublas::norm_2(yd-dr_j);
                    e(j++)=norm_2;

                    delete [] ydHelp;
                   //cout << "error " << norm_2 << std::endl;
				}
			}
			catch(std::invalid_argument& ex) // division by zero
			{
#undef max
            e(j++) = std::numeric_limits<double>::max();
            cout << "division by zero for j "<< j << std::endl;
			}
			//rank norm_inf (x)= max |xi|
            // cout << "error in residual ranking "<< e << std::endl;
			rank_vector[k++]=ublas::norm_inf(e);
			cout<<"rank value for label " << k-1 <<": "<<rank_vector[k-1]<<std::endl;

			//reset current label values
			*(get<1>($label))=1;
			*(get<2>($label))=0;

		}
		//sort the label list in the order of the sorted ranking vector
		sort($labels.begin(), $labels.end(),
			boost::lambda::var(rank_vector)[boost::lambda::bind(Li(),boost::lambda::_1)] <
			boost::lambda::var(rank_vector)[boost::lambda::bind(Li(),boost::lambda::_2)]
		);
		return $labels;

	}
	else
		throw std::runtime_error("Modelica system is not of type IReduceDAE");
}
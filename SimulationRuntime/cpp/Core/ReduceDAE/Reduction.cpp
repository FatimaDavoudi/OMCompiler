#include <Core/Modelica.h>
#include "../../Include/Core/ReduceDAE/Simulation.h"
#include "../../Include/Core/ReduceDAE/Reduction.h"
//#include "../../DataExchange/Interfaces/IHistory.h"
#include "../../Include/Core/ReduceDAE/IReduceDAESettings.h"
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/foreach.hpp>
Reduction::Reduction(IMixedSystem* system,Simulation& simulation,IReduceDAESettings* settings)
:_system(system)
,_simulation(simulation)
,_settings(settings)
{
}

Reduction::~Reduction(void)
{
}

//choose reduction method
/*std::vector<unsigned int> Reduction::DoReduction(label_list_type& labels,ublas::matrix<double>& Ro)
 {
	if(_settings->getReductionMethod() == IReduceDAESettings::CANCEL_TERMS)
			return cancelTerms(labels,Ro);
    else if(_settings->getReductionMethod() == IReduceDAESettings::LINEARIZE_TERMS)
			return linearizeTerms(labels,Ro);
	else if(_settings->getReductionMethod() == IReduceDAESettings::SUBSTITUTE_TERMS)
			return substituteTerms(labels,Ro);
	else
		throw std::runtime_error("Reduction method not supported yet!");

 }*/

//deletion
/*
std::vector<unsigned int> Reduction::cancelTerms(label_list_type& labels,ublas::matrix<double>& Ro)
{

	//if the reference matrix for output variables has no rows, then there are no output variables
	if(Ro.size1()==0)
		throw std::runtime_error("No output variables!");
	cout<<"Start deletion:" << std::endl;

	//vector of labels to be canceled
	std::vector<unsigned int> canceled_labels;
	//cast modelica system to reduce dae object
	IReduceDAE* reduce_dae = dynamic_cast<IReduceDAE*>(_system);
	//get history object to query simulation results
	IHistory* history = reduce_dae->getHistory();
	//time steps
	//unsigned long m = history->getSize();
	//simulation results for output variables of k.-reduction
	ublas::matrix<double> Rok;
	//current label
	label_type  label;

	unsigned int nfail=0;
	if(reduce_dae)
	{
		//get max error
		ublas::vector<double> max_error  = _settings->getMaxError();
		//loop over labels
		BOOST_FOREACH(label, labels)
		{
			//set current label_1 to 0 and label_2 to 1
            *(get<1>(label))=0;
            *(get<2>(label))=1;
			//clear history
			history->clear();
			//number of output variables
			ublas::matrix<double>::size_type n;
			n = Ro.size1();
            cout << "error matrix size " <<n << std::endl;
			//vector for errors
			ublas::vector<double> error (n);
			//run simulation
			_simulation.runSimulation(_system);
			//query simulation results
			history->getOutputResults(Rok);
			//cout << "before " <<Ro << std::endl;
			cout << "Rok size " <<Rok.size1() << std::endl;
            vector<string> output_names;
            history->getOutputNames(output_names);
			//error
			error= getError(Rok,Ro,output_names);
			//check if error is less than max error
			if(isLess(error,max_error))
			{
				cout << "delete term for label " << get<0>(label) << " with error "<<error<<std::endl;
				//add label number to canceled_labels
				canceled_labels.push_back(get<0>(label));
			}
			else
			{
				//check if looking for terms to reduce has failed more than allowed
				if((nfail)>_settings->getNFail())
				{
					cout << "deletion reached tolerance for label " << get<0>(label) << " with error "<<error<<std::endl;
					//stop looking for terms to delete
					break;
				}
				else
				{
					cout << "do nothing for label " << get<0>(label) << " with error "<<error<<std::endl;
					//reset current label values
					*(get<1>(label))=1;
                    *(get<2>(label))=0;
					nfail++;
				}
			}

		}

		return canceled_labels;
	}
	else
		throw std::runtime_error("Modelica system is not of type IReduceDAE");
}

std::vector<unsigned int> Reduction::linearizeTerms(label_list_type& labels,ublas::matrix<double>& Ro)
{
	//if the reference matrix for output variables has no rows, then there are no output variables
	if(Ro.size1()==0)
		throw std::runtime_error("No output variables!");
	cout<<"Start linearization:" << std::endl;

	//vector of labels to be canceled
	std::vector<unsigned int> canceled_labels;
	//cast modelica system to reduce dae object
	IReduceDAE* reduce_dae = dynamic_cast<IReduceDAE*>(_system);
	//get history object to query simulation results
	IHistory* history = reduce_dae->getHistory();
	//time steps
	//unsigned long m = history->getSize();
	//simulation results for output variables of k.-reduction
	ublas::matrix<double> Rok;
	//current label
	label_type  label;

	unsigned int nfail=0;
	if(reduce_dae)
	{
		//get max error
		ublas::vector<double> max_error  = _settings->getMaxError();
		//loop over labels
		BOOST_FOREACH(label, labels)
		{
			//set current label_1 to 0 and label_2 to 1
            *(get<1>(label))=0;
            *(get<2>(label))=1;
			//clear history
			history->clear();
			//number of output variables
			ublas::matrix<double>::size_type n;
			n = Ro.size1();
			//vector for errors
			ublas::vector<double> error (n);
			//run simulation
			_simulation.runSimulation(_system);
			//query simulation results
			history->getOutputResults(Rok);
			//cout << "before " <<Ro << std::endl;
			//cout << "after " <<Rok << std::endl;
            vector<string> output_names;
            history->getOutputNames(output_names);
			//error
			error= getError(Rok,Ro,output_names);
			//check if error is less than max error
			if(isLess(error,max_error))
			{
				cout << "linearize term for label " << get<0>(label) << " with error "<<error<<std::endl;
				//add label number to canceled_labels
				canceled_labels.push_back(get<0>(label));
			}
			else
			{
				//check if looking for terms to reduce has failed more than allowed
				if((nfail)>_settings->getNFail())
				{
					cout << "linearization reached tolerance for label " << get<0>(label) << " with error "<<error<<std::endl;
					//stop looking for terms to delete
					break;
				}
				else
				{
					cout << "do nothing for label " << get<0>(label) << " with error "<<error<<std::endl;
					//reset current label values
					*(get<1>(label))=1;
                    *(get<2>(label))=0;
					nfail++;
				}
			}

		}

		return canceled_labels;
	}
	else
		throw std::runtime_error("Modelica system is not of type IReduceDAE");
}

std::vector<unsigned int> Reduction::substituteTerms(label_list_type& labels,ublas::matrix<double>& Ro)
{
	//if the reference matrix for output variables has no rows, then there are no output variables
	if(Ro.size1()==0)
		throw std::runtime_error("No output variables!");
	cout<<"Start substitution:" << std::endl;

	//vector of labels to be canceled
	std::vector<unsigned int> canceled_labels;
	//cast modelica system to reduce dae object
	IReduceDAE* reduce_dae = dynamic_cast<IReduceDAE*>(_system);
	//get history object to query simulation results
	IHistory* history = reduce_dae->getHistory();
	//time steps
	//unsigned long m = history->getSize();
	//simulation results for output variables of k.-reduction
	ublas::matrix<double> Rok;
	//current label
	label_type  label;

	unsigned int nfail=0;
	if(reduce_dae)
	{
		//get max error
		ublas::vector<double> max_error  = _settings->getMaxError();
		//loop over labels
		BOOST_FOREACH(label, labels)
		{
			//set current label_1 to 0 and label_2 to 1
            *(get<1>(label))=0;
            *(get<2>(label))=1;
			//clear history
			history->clear();
			//number of output variables
			ublas::matrix<double>::size_type n;
			n = Ro.size1();
			//vector for errors
			ublas::vector<double> error (n);
			//run simulation
			_simulation.runSimulation(_system);
			//query simulation results
			history->getOutputResults(Rok);
			//cout << "before " <<Ro << std::endl;
			//cout << "after " <<Rok << std::endl;
            vector<string> output_names;
            history->getOutputNames(output_names);
			//error
			error= getError(Rok,Ro,output_names);
			//check if error is less than max error
			if(isLess(error,max_error))
			{
				cout << "substitute term for label " << get<0>(label) << " with error "<<error<<std::endl;
				//add label number to canceled_labels
				canceled_labels.push_back(get<0>(label));
			}
			else
			{
				//check if looking for terms to reduce has failed more than allowed
				if((nfail)>_settings->getNFail())
				{
					cout << "substitution reached tolerance for label " << get<0>(label) << " with error "<<error<<std::endl;
					//stop looking for terms to delete
					break;
				}
				else
				{
					cout << "do nothing for label " << get<0>(label) << " with error "<<error<<std::endl;
					//reset current label values
					*(get<1>(label))=1;
                    *(get<2>(label))=0;
					nfail++;
				}
			}

		}

		return canceled_labels;
	}
	else
		throw std::runtime_error("Modelica system is not of type IReduceDAE");
}*/

//find difference between reference and current output values
ublas::vector<double> Reduction::getError(ublas::matrix<double>& R,ublas::matrix<double>& R2,vector<string> output_names)
{
	//using namespace boost::math;
	ublas::matrix<double>::size_type i,n;
	ublas::vector<double> o_i;
	ublas::vector<double> or_i;
	unsigned counter=0;
	//number of output values
	n = R.size1();
	//vector for error
    ublas::vector<double> error(n);


	for(i=0;i<n;++i)
	{
		//vector for current output values
		o_i=ublas::row(R,i);
        //cout << "after reduction: Output variable "<<output_names[counter]<< " "<< o_i<<std::endl;
		//vector for reference output values
		or_i=ublas::row(R2,i);
        //cout << "original Output variable "<<output_names[counter]<< " "<< or_i<<std::endl;
		//check if both vectors have the same number of entries
		if(o_i.size()==or_i.size())
        {
            // cout << "subtraction of reduced values from originals "<<output_names[counter]<< " "<< sum(o_i-or_i)/n <<std::endl;
             //norm_2 (x)= sqrt (sum |xi|^2 )
            error(i)=ublas::norm_2(o_i-or_i);
          // cout << "Error by reduction: Output variable "<<output_names[counter]<< " "<< error(i) <<std::endl;
        }
		//if o_i and or_i are of different size, then simulation stopped - label should not be removed: error gets max possible value
		#undef max
		else
			error(i) = std::numeric_limits<double>::max( );

        if(boost::math::isnan(error(i)))
			error(i) = std::numeric_limits<double>::max( );
         counter++;
	    }


	return error;

}

//function for checking if error is less than max error
bool Reduction::isLess(ublas::vector<double>& v1,ublas::vector<double>& v2, vector<int> indexes,vector<string> output_names)
{
 //cout << " v1 "<<v1<<std::endl;
//cout << " sorted_max_error "<<v2<<std::endl;
	//check if number of output variables corresponds to the number of output variables in ReduceDAESettings.xml
    //if(v1.size() == v2.size())
	//{
		ublas::vector<double>::iterator iter,iter2;
		//check if error is less than max error
		/*for(iter=v1.begin(),iter2=v2.begin();iter!=v1.end();++iter,++iter2)
        {if((*iter)>=(*iter2))
        return false;
        }*/
        for(int i=0;i< indexes.size();i++)
		{
           cout<<indexes[i]<<" error variable "<<output_names[indexes[i]]<<" with error bound "<<v2[i]<<" is "<< v1[indexes[i]]<<std::endl;
           if((v1[indexes[i]]>= v2[i]) && v2[i]!=0)
				return false;
            else if (v2[i]==0 && (v1[indexes[i]]!= v2[i]))
                return false;
		}
		return true;
	//}
	//else{throw std::runtime_error("Number of output variables does not correspond to ReduceDAESettings.xml!"); }
}
#include <Core/Modelica.h>
#include "../../Include/Core/ReduceDAE/Simulation.h"
#include "../../Include/Core/Math/Constants.h"
//#include "../../Include/Core/System/IMixedSystem.h"
//#include "../../Include/Core/System/IEvent.h"
//#include "../../Include/Core/System/ISystemProperties.h"
#include <boost/bind.hpp>
Simulation::Simulation(IMixedSystem* system,Configuration& config)
:_system(system)
,_config(config)
{

	IEvent* event_system = dynamic_cast<IEvent*>(_system);
	int dim = event_system->getDimZeroFunc();
	_events	= new bool[dim];
	memset(_events,false,dim*sizeof(bool));

	_zeroValLastSuccess	= new double[dim];
	_zeroVal = new double[dim];
	//event_system->giveZeroFunc(_zeroVal,1e-6);
	event_system->getZeroFunc(_zeroVal);
	memcpy(_zeroValLastSuccess,_zeroVal,dim*sizeof(double));

}

Simulation::~Simulation(void)
{
	if(_events)
		delete [] _events;
	if(_zeroValLastSuccess)
		delete [] _zeroValLastSuccess;
	if(_zeroVal)
		delete [] _zeroVal;
}


bool Simulation::checkZeroState()
{
	IEvent* event_system = dynamic_cast<IEvent*>(_system);
	// Reset Zero-State
	bool event_iteration = false;

     event_system->getZeroFunc(_zeroVal);
	int dim = event_system->getDimZeroFunc();
	// For all zero functions...
	for (int i=0; i<dim; ++i)
	{
		// Check for change in sign in each zero function
		if (_zeroVal[i] * _zeroValLastSuccess[i] <= 0.0 && fabs(_zeroVal[i]-_zeroValLastSuccess[i]) > UROUND)
		{
			// EQUAL_ZERO
			//-----------
			// Check whether value zero function is smaller than tolerance OR step size is smaller than time-tolerance
			if ( (fabs(_zeroVal[i])) < /*_settings->_zeroTol*/ 1e-4 )
			{
				event_iteration = true;

				// Store which zero function caused event
				_events[i] = true; //_zeroSign = sgn(_zeroVal[i]-_zeroValLastSuccess[i]);

				// zeroVal is not allowed to be =0, since otherwise the direction of sign change cannot be determined in next step
				if ( _zeroVal[i] == 0.0 )
					_zeroVal[i] = -sgn(_zeroValLastSuccess[i]) * UROUND;
			}

			// ZERO_CROSSING
			//--------------
			else
			{
				// Change in sign, but zeroVal is not smaller than given tolerance


				// Reset zeroSign
				_events[i] = false;

				break;
			}
		}

		// UNCHANGED_SIGN
		//----------------
		else
		{
			_events[i] = false;
			event_system->getZeroFunc(_zeroValLastSuccess);

		}


	}
     if(event_iteration)
	 {

	        event_system->handleEvent(_events);
	 }
	 return event_iteration;

}

void Simulation::updateEventState()
{

}
void Simulation::runSimulation(IMixedSystem* system)
{
    try
	{
		cout << "here is simulation run func " << std::endl;
	    ISolver* solver = _config.createSelectedSolver(system).get();
        cout << "createSelectedSolver " << std::endl;
		ISystemProperties* system_properties = dynamic_cast<ISystemProperties*>(system);
        cout << "system_properties " << std::endl;

		if((system_properties->isODE()) && !(system_properties->isAlgebraic()))
		{
             cout << "inside if " << std::endl;
			// Command for integration: Since integration is done "at once" the solver is only called once. Hence it is both, first and last
			// call to the solver at the same time. Furthermore it is supposed to be a regular call (not a recall)
			ISolver::SOLVERCALL command = ISolver::SOLVERCALL(ISolver::FIRST_CALL);
            cout << "SOLVERCALL " << std::endl;
			// The simulation entity is supposed to set start and end time
			solver->setStartTime(_config.getGlobalSettings()->getStartTime());
            cout << "setStartTime " << std::endl;
			solver->setEndTime(_config.getGlobalSettings()->getEndTime());
            cout << "setEndTime " << std::endl;
			solver->setInitStepSize(_config.getSolverSettings()->gethInit());
            cout << "setInitStepSize " << std::endl;
			// Call the solver
			solver->solve(command);
            cout << "solve " << std::endl;
		}
		// Get the status of the solver (is the interation done sucessfully?)
		ISolver::SOLVERSTATUS status = solver->getSolverStatus();
        cout << "SOLVERSTATUS " << std::endl;
		solver->writeSimulationInfo();

		cout << "writeSimulationInfo is done " << std::endl;
		//solver->reportErrorMessage(std::cout);
	}
	catch(std::exception& ex)
	{
		std::string error = ex.what();
		std::cout << "Simulation stopped: "<< std::endl << error << std::endl;
	}





}

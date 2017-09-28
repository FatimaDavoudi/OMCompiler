#pragma once
//#include "../../Include/Core/System/IMixedSystem.h"
//#include "../../Include/Core/Solver/ISolverSettings.h"
#include <Core/SimController/Configuration.h>
class Simulation
{
public:
	Simulation(IMixedSystem* system,Configuration& config);
	~Simulation(void);
	bool checkZeroState();
	void runSimulation(IMixedSystem* system);
	void updateEventState();
private:
	Configuration& _config;
    IMixedSystem*  _system;
	bool*	_events;
	/// Definition of signum function
	inline static int sgn (const double &c)
	{
		return (c < 0) ? -1 : ((c == 0) ? 0 : 1);
	}

	double	*_zeroVal;
	double	*_zeroValLastSuccess;
};

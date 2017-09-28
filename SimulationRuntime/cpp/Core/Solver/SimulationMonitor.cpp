/** @addtogroup coreSolver
 *
 *  @{
 */
#include <Core/ModelicaDefine.h>
#include <Core/Modelica.h>
#include <Core/Solver/FactoryExport.h>
#include <Core/Solver/SimulationMonitor.h>
/*_time_out(0)
   ,*/
SimulationMonitor::SimulationMonitor()
  :_time_out(0)
   ,_interrupt(false)
{
}

SimulationMonitor::~SimulationMonitor()
{
}

void SimulationMonitor::initialize()
{
    //being uncomment for labeling reduction
  _timer = cpu_timer();
  _interrupt = false;
}

void SimulationMonitor::setTimeOut(double time_out)
{

    double tmp = time_out * 1e9;
     unsigned int tmp_int = tmp;
   //cout<<"tmp_in "<< tmp_int <<std::endl;
   //being uncomment for labeling reduction
   _time_out = nanosecond_type(tmp_int);
}
void SimulationMonitor::stop()
{
  _interrupt =true;
}
void SimulationMonitor::checkTimeout()
{
    //The whole inside code, being uncomment for labeling reduction
   cpu_times  elapsed_times(_timer.elapsed());
  nanosecond_type elapsed(elapsed_times.system  + elapsed_times.user);
  if (_time_out>0 && elapsed >= _time_out)
  {
  _interrupt =true;
  }

}
 /** @} */ // end of coreSolver

/** @addtogroup coreSimcontroller
 *
 *  @{
 */
#include <Core/ModelicaDefine.h>
#include <Core/Modelica.h>
#include <Core/SimController/ISimController.h>
#include <Core/SimController/SimController.h>
#include <Core/SimController/Configuration.h>
#include <Core/SimController/SimObjects.h>
#include <core/ReduceDAE/ReduceDAESettings.h>
#include <core/ReduceDAE/Ranking.h>
#include <core/ReduceDAE/Reduction.h>
#include <core/ReduceDAE/Simulation.h>
#include <core/ReduceDAE/com/ModelicaCompiler.h>
#include <boost/foreach.hpp>
#if defined(OMC_BUILD) || defined(SIMSTER_BUILD)
#include "LibrariesConfig.h"
#endif


SimController::SimController(PATH library_path, PATH modelicasystem_path)
    : SimControllerPolicy(library_path, modelicasystem_path, library_path)
    , _initialized(false)
{
    _config = shared_ptr<Configuration>(new Configuration(_library_path, _config_path, modelicasystem_path));
    _sim_objects = shared_ptr<ISimObjects>(new SimObjects(_library_path,modelicasystem_path,_config->getGlobalSettings().get()));

    #ifdef RUNTIME_PROFILING
    measuredFunctionStartValues = NULL;
    measuredFunctionEndValues = NULL;

    if(MeasureTime::getInstance() != NULL)
    {
        measureTimeFunctionsArray = new std::vector<MeasureTimeData*>(2, NULL); //0 initialize //1 solveInitialSystem
        (*measureTimeFunctionsArray)[0] = new MeasureTimeData("initialize");
        (*measureTimeFunctionsArray)[1] = new MeasureTimeData("solveInitialSystem");

        measuredFunctionStartValues = MeasureTime::getZeroValues();
        measuredFunctionEndValues = MeasureTime::getZeroValues();
    }
    else
    {
      measureTimeFunctionsArray = new std::vector<MeasureTimeData*>();
    }
    #endif
}

SimController::~SimController()
{
    #ifdef RUNTIME_PROFILING
    if(measuredFunctionStartValues)
      delete measuredFunctionStartValues;
    if(measuredFunctionEndValues)
      delete measuredFunctionEndValues;
    #endif
}

weak_ptr<IMixedSystem> SimController::LoadSystem(string modelLib,string modelKey)
{

    //if the model is already loaded
    std::map<string,shared_ptr<IMixedSystem> >::iterator iter = _systems.find(modelKey);
    if(iter != _systems.end())
    {
        _sim_objects->eraseSimData(modelKey);
        _sim_objects->eraseSimVars(modelKey);
        //destroy system
        _systems.erase(iter);
    }
     //create system
    shared_ptr<IMixedSystem> system = createSystem(modelLib, modelKey, _config->getGlobalSettings().get(), _sim_objects);
    _systems[modelKey] = system;
    return system;
}

weak_ptr<IMixedSystem> SimController::LoadModelicaSystem(PATH modelica_path,string modelKey)
{
    if(_use_modelica_compiler)
    {
        // if the modell is already loaded
        std::map<string,shared_ptr<IMixedSystem> >::iterator iter = _systems.find(modelKey);
        if(iter != _systems.end())
        {
            _sim_objects->eraseSimData(modelKey);
            _sim_objects->eraseSimVars(modelKey);
            // destroy system
            _systems.erase(iter);
        }

        shared_ptr<IMixedSystem> system = createModelicaSystem(modelica_path, modelKey, _config->getGlobalSettings().get(),_sim_objects);
        _systems[modelKey] = system;
        return system;
    }
    else
        throw ModelicaSimulationError(SIMMANAGER,"No Modelica Compiler configured");
}


 shared_ptr<ISimObjects> SimController::getSimObjects()
 {

    return _sim_objects;

 }

shared_ptr<IMixedSystem> SimController::getSystem(string modelname)
{
    std::map<string,shared_ptr<IMixedSystem> >::iterator iter = _systems.find(modelname);
    if(iter!=_systems.end())
    {
        return iter->second;
    }
    else
    {
        string error = string("Simulation data was not found for model: ") + modelname;
        throw ModelicaSimulationError(SIMMANAGER,error);
    }
}



// Added for real-time simulation using VxWorks and Bodas
void SimController::StartVxWorks(SimSettings simsettings,string modelKey)
{
    try
    {
        shared_ptr<IMixedSystem> mixedsystem = getSystem(modelKey);
         shared_ptr<IGlobalSettings> global_settings = _config->getGlobalSettings();

        global_settings->useEndlessSim(true);
        global_settings->setStartTime(simsettings.start_time);
        global_settings->setEndTime(simsettings.end_time);
        global_settings->sethOutput(simsettings.step_size);
        global_settings->setResultsFileName(simsettings.outputfile_name);
        global_settings->setSelectedLinSolver(simsettings.linear_solver_name);
        global_settings->setSelectedNonLinSolver(simsettings.nonlinear_solver_name);
        global_settings->setSelectedSolver(simsettings.solver_name);
        global_settings->setAlarmTime(simsettings.timeOut);
        global_settings->setLogSettings(simsettings.logSettings);
        global_settings->setOutputPointType(simsettings.outputPointType);
        global_settings->setOutputFormat(simsettings.outputFormat);
        global_settings->setEmitResults(simsettings.emitResults);
        /*shared_ptr<SimManager>*/ _simMgr = shared_ptr<SimManager>(new SimManager(mixedsystem, _config.get()));

        ISolverSettings* solver_settings = _config->getSolverSettings();
        solver_settings->setLowerLimit(simsettings.lower_limit);
        solver_settings->sethInit(simsettings.lower_limit);
        solver_settings->setUpperLimit(simsettings.upper_limit);
        solver_settings->setRTol(simsettings.tolerance);
        solver_settings->setATol(simsettings.tolerance);

        _simMgr->initialize();
    }
    catch( ModelicaSimulationError& ex)
    {
        string error = add_error_info(string("Simulation failed for ") + simsettings.outputfile_name,ex.what(),ex.getErrorID());
        printf("Fehler %s\n", error.c_str());
        throw ModelicaSimulationError(SIMMANAGER,error);
    }
}

// Added for real-time simulation using VxWorks and Bodas
void SimController::calcOneStep()
{
    _simMgr->runSingleStep();
}

void SimController::Start(SimSettings simsettings, string modelKey)
{
    try
    {
        #ifdef RUNTIME_PROFILING
        MEASURETIME_REGION_DEFINE(simControllerInitializeHandler, "SimControllerInitialize");
        MEASURETIME_REGION_DEFINE(simControllerSolveInitialSystemHandler, "SimControllerSolveInitialSystem");
        if(MeasureTime::getInstance() != NULL)
        {
            MEASURETIME_START(measuredFunctionStartValues, simControllerInitializeHandler, "CVodeWriteOutput");
        }
        #endif
        shared_ptr<IMixedSystem> mixedsystem = getSystem(modelKey);

        shared_ptr<IGlobalSettings> global_settings = _config->getGlobalSettings();

        global_settings->setStartTime(simsettings.start_time);
        global_settings->setEndTime(simsettings.end_time);
        global_settings->sethOutput(simsettings.step_size);
        global_settings->setResultsFileName(simsettings.outputfile_name);
        global_settings->setSelectedLinSolver(simsettings.linear_solver_name);
        global_settings->setSelectedNonLinSolver(simsettings.nonlinear_solver_name);
        global_settings->setSelectedSolver(simsettings.solver_name);
        global_settings->setLogSettings(simsettings.logSettings);
        global_settings->setAlarmTime(simsettings.timeOut);
        global_settings->setOutputPointType(simsettings.outputPointType);
        global_settings->setOutputFormat(simsettings.outputFormat);
        global_settings->setEmitResults(simsettings.emitResults);
        global_settings->setNonLinearSolverContinueOnError(simsettings.nonLinearSolverContinueOnError);
        global_settings->setSolverThreads(simsettings.solverThreads);
        /*shared_ptr<SimManager>*/ _simMgr = shared_ptr<SimManager>(new SimManager(mixedsystem, _config.get()));

        ISolverSettings* solver_settings = _config->getSolverSettings();
        solver_settings->setLowerLimit(simsettings.lower_limit);
        solver_settings->sethInit(simsettings.lower_limit);
        solver_settings->setUpperLimit(simsettings.upper_limit);
        solver_settings->setRTol(simsettings.tolerance);
        solver_settings->setATol(simsettings.tolerance);
        #ifdef RUNTIME_PROFILING
        if(MeasureTime::getInstance() != NULL)
        {
            MEASURETIME_END(measuredFunctionStartValues, measuredFunctionEndValues, (*measureTimeFunctionsArray)[0], simControllerInitializeHandler);
            measuredFunctionStartValues->reset();
            measuredFunctionEndValues->reset();
            MEASURETIME_START(measuredFunctionStartValues, simControllerSolveInitialSystemHandler, "SolveInitialSystem");
        }
        #endif

        _simMgr->initialize();

        #ifdef RUNTIME_PROFILING
        if(MeasureTime::getInstance() != NULL)
        {
            MEASURETIME_END(measuredFunctionStartValues, measuredFunctionEndValues, (*measureTimeFunctionsArray)[1], simControllerSolveInitialSystemHandler);
            MeasureTime::addResultContentBlock(mixedsystem->getModelName(),"simController",measureTimeFunctionsArray);
        }
        #endif

        _simMgr->runSimulation();

		if(global_settings->getOutputFormat() == BUFFER)
		{
			shared_ptr<IWriteOutput> writeoutput_system = dynamic_pointer_cast<IWriteOutput>(mixedsystem);

			shared_ptr<ISimData> simData = _sim_objects->getSimData(modelKey);
			simData->clearResults();
			//get history object to query simulation results
			IHistory* history = writeoutput_system->getHistory();
			//simulation results (output variables)
			ublas::matrix<double> Ro;
			//query simulation result outputs
			history->getOutputResults(Ro);
			vector<string> output_names;
			history->getOutputNames(output_names);
			int j=0;

			FOREACH(string& name, output_names)
			{
				ublas::vector<double> o_j;
				o_j = ublas::row(Ro,j);
				simData->addOutputResults(name,o_j);
				j++;
			}

			vector<double> time_values = history->getTimeEntries();
			simData->addTimeEntries(time_values);
		}
    }
    catch(ModelicaSimulationError & ex)
    {
        string error = add_error_info(string("Simulation failed for ") + simsettings.outputfile_name,ex.what(),ex.getErrorID());
        throw ModelicaSimulationError(SIMMANAGER, error, "", ex.isSuppressed());
    }
}

void SimController::StartReduceDAE(SimSettings simsettings,string modelPath, string modelKey,bool loadMSL, bool loadPackage)
{
    try
    {

         #ifdef RUNTIME_PROFILING
        MEASURETIME_REGION_DEFINE(simControllerInitializeHandler, "SimControllerInitialize");
        MEASURETIME_REGION_DEFINE(simControllerSolveInitialSystemHandler, "SimControllerSolveInitialSystem");
        if(MeasureTime::getInstance() != NULL)
        {
            MEASURETIME_START(measuredFunctionStartValues, simControllerInitializeHandler, "CVodeWriteOutput");
        }
        #endif
        shared_ptr<IMixedSystem> mixedsystem = getSystem(modelKey);

        shared_ptr<IGlobalSettings> global_settings = _config->getGlobalSettings();

        global_settings->setStartTime(simsettings.start_time);
        global_settings->setEndTime(simsettings.end_time);
        global_settings->sethOutput(simsettings.step_size);
        global_settings->setResultsFileName(simsettings.outputfile_name);
        global_settings->setSelectedLinSolver(simsettings.linear_solver_name);
        global_settings->setSelectedNonLinSolver(simsettings.nonlinear_solver_name);
        global_settings->setSelectedSolver(simsettings.solver_name);
        global_settings->setLogSettings(simsettings.logSettings);
        global_settings->setAlarmTime(simsettings.timeOut);
        // global_settings->setAlarmTime(2);
        global_settings->setOutputPointType(simsettings.outputPointType);
        global_settings->setOutputFormat(simsettings.outputFormat);
        global_settings->setEmitResults(simsettings.emitResults);
        global_settings->setNonLinearSolverContinueOnError(simsettings.nonLinearSolverContinueOnError);
        global_settings->setSolverThreads(simsettings.solverThreads);
        /*shared_ptr<SimManager>*/ _simMgr = shared_ptr<SimManager>(new SimManager(mixedsystem, _config.get()));

        ISolverSettings* solver_settings = _config->getSolverSettings();
        solver_settings->setLowerLimit(simsettings.lower_limit);
        solver_settings->sethInit(simsettings.lower_limit);
        solver_settings->setUpperLimit(simsettings.upper_limit);
        solver_settings->setRTol(simsettings.tolerance);
        solver_settings->setATol(simsettings.tolerance);
        #ifdef RUNTIME_PROFILING
        if(MeasureTime::getInstance() != NULL)
        {
            MEASURETIME_END(measuredFunctionStartValues, measuredFunctionEndValues, (*measureTimeFunctionsArray)[0], simControllerInitializeHandler);
            measuredFunctionStartValues->reset();
            measuredFunctionEndValues->reset();
            MEASURETIME_START(measuredFunctionStartValues, simControllerSolveInitialSystemHandler, "SolveInitialSystem");
        }
        #endif
        auto startSim1 = std::chrono::high_resolution_clock::now();
        _simMgr->initialize();

        #ifdef RUNTIME_PROFILING
        if(MeasureTime::getInstance() != NULL)
        {
            MEASURETIME_END(measuredFunctionStartValues, measuredFunctionEndValues, (*measureTimeFunctionsArray)[1], simControllerSolveInitialSystemHandler);
            MeasureTime::addResultContentBlock(mixedsystem->getModelName(),"simController",measureTimeFunctionsArray);
        }
        #endif


		_simMgr->runSimulation();
         auto endSim1 = std::chrono::high_resolution_clock::now();
         double timeout=std::chrono::duration_cast<std::chrono::duration<double>>(endSim1-startSim1).count();
         std::cout <<" time of first simulation: "<< timeout << " seconds" << std::endl;
        IReduceDAE* reduce_dae = dynamic_cast<IReduceDAE*>(mixedsystem.get());
        if(reduce_dae==NULL)
		{
			throw std::runtime_error("Modelica System is not of type IReduceDAE!!!");
		}

        //read reduced settings
        ReduceDAESettings reduce_settings(global_settings.get());
		reduce_settings.load("ReduceDAESettings.xml");

		//get history object to query simulation results
		IHistory* history = reduce_dae->getHistory();
        vector<double> time_values = history->getTimeEntries();
         cout << "time_values: " << time_values.size() << std::endl;

		//simulation results (algebraic and state variables)
		ublas::matrix<double> R;
		//simulation results (derivative variables)
		ublas::matrix<double> dR;
		//simulation results (residues)
		ublas::matrix<double> Re;
		//simulation results (output variables)
		ublas::matrix<double> Ro;
		//query simulation results
		history->getSimResults(R,dR,Re);

         cout << "number of derivatives: " << dR.size1() << std::endl;
        cout << "number of variables: " << R.size1() << std::endl;
        cout << "number of residual: " << Re.size1() << std::endl;
		history->getOutputResults(Ro);
       cout << "number of output " << Ro.size1() << std::endl;
        vector<string> output_names;
		 history->getOutputNames(output_names);


        label_list_type labels;


        //----------------------------------------------------------------------------------------------------------------
         //start  ranking
        Ranking ranking(mixedsystem.get(),&reduce_settings);

        if(reduce_settings.getRankingMethod()==IReduceDAESettings::RESIDUEN)
        {

        auto start = std::chrono::high_resolution_clock::now();
        //start  residue ranking
          labels =ranking.residuenRanking(R,dR,Re,time_values);
         auto end = std::chrono::high_resolution_clock::now();
        std::cout <<" time of residual ranking: "<< std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << " milliseconds" << std::endl;

        }


        else if(reduce_settings.getRankingMethod()==IReduceDAESettings::PERFECT)
        {
          //start  perfect ranking
           auto start = std::chrono::high_resolution_clock::now();
         labels = perfectRanking(Ro,mixedsystem,&reduce_settings,simsettings,modelKey,output_names,timeout);
           auto end = std::chrono::high_resolution_clock::now();
        std::cout <<" time of perfect ranking: "<< std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << " milliseconds" << std::endl;
        }


        else if(reduce_settings.getRankingMethod()==IReduceDAESettings::NORANKING)
        {
            //without ranking, get all labels without any sorting and pass it to the redction
            labels=reduce_dae->getLabels();
        }

        label_type  label;
        std::cout<<"sorted labels: "<<"\n";
        BOOST_FOREACH(label, labels)
		{
            std::cout<<"label "<< get<0>(label)<<"\n";

        }



         //----------------------------------------------------------------------------------------------------------------
         //start reduction
		std::vector<unsigned int> terms = cancelTerms(labels,Ro,mixedsystem,&reduce_settings,simsettings,modelKey,output_names,timeout);

        if(terms.size()>0)
        {

         //------------------------------------------------------------------------------------
          string packageName;
          size_t found;
           std::cout << "modelPath "<< modelPath << std::endl;
          found=modelPath.find(modelKey);
          //std::cout << "found "<< found << std::endl;

        if (found != std::string::npos && found!=0)
            packageName=modelPath.substr(0, found-1);
		else
            packageName="";
        std::cout << "package name "<< packageName<< std::endl;
         string fileName = modelKey;
         ModelicaCompiler* compiler;
         // when a model from MSL is used, then LoadFile doesn't need to be called
         // still there is problem with calling reducedTerms with model from MSL, because
         // for example for Modelica.Electrical.Analog.Examples.CauerLowPassSC, the modelPath only gives Modelica.CauerLowPassSC
        compiler =new ModelicaCompiler(modelKey,fileName,packageName,!loadMSL,loadPackage);
        compiler->reduceTerms(terms,simsettings.start_time,simsettings.end_time);
        //-----------------------------------------------------------------------------------------


        /*_simMgr->runSimulation();
        }*/

        }

         else
         std::cout << "list of labels for reduction is empty, so model remained as original." << std::endl;


    }
    catch(ModelicaSimulationError & ex)
    {
        string error = add_error_info(string("Simulation failed for ") + simsettings.outputfile_name,ex.what(),ex.getErrorID());
        throw ModelicaSimulationError(SIMMANAGER, error, "", ex.isSuppressed());
    }
}

std::vector<unsigned int> SimController::cancelTerms(label_list_type& labels,ublas::matrix<double>& Ro, shared_ptr<IMixedSystem> _system,IReduceDAESettings* _settings,SimSettings simsettings, string modelKey,vector<string> output_names,double timeout)
{

	//if the reference matrix for output variables has no rows, then there are no output variables
	if(Ro.size1()==0)
		throw std::runtime_error("No output variables!");
	cout<<"Start deletion:" << std::endl;

	//vector of labels to be canceled
	std::vector<unsigned int> canceled_labels;
    std::vector<unsigned int> help_canceled_labels;
	//cast modelica system to reduce dae object

	IReduceDAE* reduce_dae=dynamic_cast<IReduceDAE*>(_system.get());
	//get history object to query simulation results
	IHistory*  history;

	//simulation results for output variables of k.-reduction
	ublas::matrix<double> Rok;
	//current label
	label_type  label;

	unsigned int nfail=0;
    unsigned int reductionStep=1;
	if(reduce_dae)
	{

		//get max error
		ublas::vector<double> max_error  = _settings->getMaxError();
       vector<string> output_names_xml = _settings->getOutputNames();
       ublas::vector<double> sorted_max_error(max_error.size());
       vector<int> indexes;
        ublas::vector<double>::size_type  i=0;
        //make sorted vector of error based on given variables name from buffer
       for(int j=0; j<output_names.size();j++)
		{
      auto it = std::find(output_names_xml.begin(), output_names_xml.end(), output_names[j]);
        if (it == output_names_xml.end())
        {
            // name not in vector
        } else
        {
           auto index = std::distance(output_names_xml.begin(), it);
           sorted_max_error.insert_element(i,max_error[index]);
           i++;
            indexes.push_back(j);
        }
            }

        cout << "sorted_max_error " <<sorted_max_error<<std::endl;

        auto start = std::chrono::high_resolution_clock::now();

		//loop over labels
		BOOST_FOREACH(label, labels)
		{
            try{

            //removing initialization from here and put it ouside of BOOST_FOREACH,
            //cause not updated output values on Rok vector!!
             auto startSim1 = std::chrono::high_resolution_clock::now();
            initialize(simsettings, modelKey,timeout);
			//set current label_1 to 0 and label_2 to 1
            *(get<1>(label))=0;
            *(get<2>(label))=1;
            //by initialization all labels becomes 1,
            // so with this help_canceled_labels we apply all labels which are zero untill now to the model again
        for(int i=0; i< help_canceled_labels.size();i++)
        {

            //cout << "indexes of deleted labels applied to the model " <<  get<0>(labels[help_canceled_labels[i]]) <<std::endl;
              *(get<1>(labels[help_canceled_labels[i]]))=0;
              *(get<2>(labels[help_canceled_labels[i]]))=1;
        }

			//number of output variables
			ublas::matrix<double>::size_type n;
			n = Ro.size1();
			//vector for errors
			ublas::vector<double> error (n);
			//run simulation
            _simMgr->runSimulation();
            auto endSim1 = std::chrono::high_resolution_clock::now();
         double simtime=std::chrono::duration_cast<std::chrono::duration<double>>(endSim1-startSim1).count();
         std::cout <<" time of simulation for reducing label "<< get<0>(label)<<" is " <<simtime << " seconds" << std::endl;
            history = reduce_dae->getHistory();

            //query simulation result outputs
            Rok.clear();
			history->getOutputResults(Rok);

			//error
            Simulation simulation_labelled(_system.get(),*_config);
            Reduction reduction(_system.get(),simulation_labelled,_settings);

			error= reduction.getError(Rok,Ro,output_names);

			//check if error of selected varibles based on indexes vector is less than max error
			if(reduction.isLess(error,sorted_max_error,indexes,output_names))
			{
				cout << "delete term for label " << get<0>(label) << " with error "<<error<<std::endl;
				//add label number to canceled_labels
				canceled_labels.push_back(get<0>(label));
                help_canceled_labels.push_back(reductionStep-1);
			}
			else
			{
					cout << "do nothing for label " << get<0>(label) << " with error "<<error<<std::endl;
					//reset current label values
					*(get<1>(label))=1;
                    *(get<2>(label))=0;
					nfail++;

                    //check if looking for terms to reduce has failed more than allowed
				if((nfail)>_settings->getNFail())
				{
					cout << "Redution stoped at step " << reductionStep+1 <<" because of exceeding max number of reduction fails"<<std::endl;
					//stop looking for terms to delete
					break;
				}

			}

            }

             catch(ModelicaSimulationError& ex)
            {
                if(!ex.isSuppressed())
                     cout << "do nothing for label " << get<0>(label) << " with error "<<ex.what()<<std::endl;
                   // std::cerr << "Simulation stopped with error in " << error_id_string(ex.getErrorID()) << ": "  << ex.what();
                   //reset current label values
					*(get<1>(label))=1;
                    *(get<2>(label))=0;
					nfail++;

                    //check if looking for terms to reduce has failed more than allowed
				if((nfail)>_settings->getNFail())
				{
					cout << "Redution failed for "<<nfail<<" times. So, it stoped at step " << reductionStep+1 <<std::endl;
					//stop looking for terms to delete
					break;
				}

            }

			reductionStep++;

		}
         auto end = std::chrono::high_resolution_clock::now();
        std::cout <<" time of reduction: "<< std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << " milliseconds" << std::endl;
		return canceled_labels;
	}
	else
		throw std::runtime_error("Modelica system is not of type IReduceDAE");
}

label_list_type SimController::perfectRanking(ublas::matrix<double>& Ro,shared_ptr<IMixedSystem> _system,IReduceDAESettings* _settings,SimSettings simsettings, string modelKey,vector<string> output_names, double timeout)
{
    ublas::matrix<double>::size_type n;
			n = Ro.size1();
            if(n==0)
		throw std::runtime_error("No output variables for ranking!");
	cout<<"Start perfect ranking:" << std::endl;
	//cast modelica system to reduce dae object
	IReduceDAE* reduce_dae = dynamic_cast<IReduceDAE*>(_system.get());

	IHistory* history;
	ublas::matrix<double> Rc; //current result

	if(reduce_dae)
	{

        label_type  label;
		//get label variables
		label_list_type labels = reduce_dae->getLabels();
		//number of labels
		int kDim = labels.size();
		//vector for ranks
		vector<double> rank_vector(kDim);
         double rank_value;
         //index for current ranking
		unsigned int k=0;
		//loop over labels
		BOOST_FOREACH(label, labels)
		{
            try{

            initialize(simsettings, modelKey,timeout);
				 //set label_1 to 0 and label_2 to 1
                    *(get<1>(label))=0;
                    *(get<2>(label))=1;
                  //vector for errors
                    ublas::vector<double> e(n);

             _simMgr->runSimulation();
            history = reduce_dae->getHistory();
            //query simulation result outputs
             Rc.clear();
			history->getOutputResults(Rc);

             Simulation simulation_labelled(_system.get(),*_config);
            Reduction reduction(_system.get(),simulation_labelled,_settings);
             e=reduction.getError(Rc,Ro,output_names);;

            //rank norm_inf (x)= max |xi|
             rank_value=ublas::norm_inf(e);

            }
             catch(ModelicaSimulationError& ex)
            {
              #undef max
              rank_value = std::numeric_limits<double>::max();
                if(!ex.isSuppressed())
                 cout << "removing label "<<(get<0>(label))<< "causes error "<<ex.what()<< std::endl;
               // std::cerr << "Simulation stopped with error in " << error_id_string(ex.getErrorID()) << ": "  << ex.what();

            }
            catch(std::invalid_argument& ex) // division by zero
			{
            #undef max
            rank_value = std::numeric_limits<double>::max();
            cout << "division by zero for label "<<(get<0>(label))<< std::endl;
			}



			rank_vector[k++]=rank_value;
			cout<<"rank value for label " << (get<0>(label)) <<": "<<rank_value<<std::endl;

			//reset current label values
			*(get<1>(label))=1;
			*(get<2>(label))=0;
		}
		//sort the label list in the order of the sorted ranking vector
		sort(labels.begin(), labels.end(),
			boost::lambda::var(rank_vector)[boost::lambda::bind(Li(),boost::lambda::_1)] <
			boost::lambda::var(rank_vector)[boost::lambda::bind(Li(),boost::lambda::_2)]
		);
		return labels;

	}
	else
		throw std::runtime_error("Modelica system is not of type IReduceDAE");
}

void SimController::initialize(SimSettings simsettings, string modelKey, double timeout)
{
    try
    {
        #ifdef RUNTIME_PROFILING
        MEASURETIME_REGION_DEFINE(simControllerInitializeHandler, "SimControllerInitialize");
        MEASURETIME_REGION_DEFINE(simControllerSolveInitialSystemHandler, "SimControllerSolveInitialSystem");
        if(MeasureTime::getInstance() != NULL)
        {
            MEASURETIME_START(measuredFunctionStartValues, simControllerInitializeHandler, "CVodeWriteOutput");
        }
        #endif
        shared_ptr<IMixedSystem> mixedsystem = getSystem(modelKey);

        shared_ptr<IGlobalSettings> global_settings = _config->getGlobalSettings();

        global_settings->setStartTime(simsettings.start_time);
        global_settings->setEndTime(simsettings.end_time);
        global_settings->sethOutput(simsettings.step_size);
        global_settings->setResultsFileName(simsettings.outputfile_name);
        global_settings->setSelectedLinSolver(simsettings.linear_solver_name);
        global_settings->setSelectedNonLinSolver(simsettings.nonlinear_solver_name);
        global_settings->setSelectedSolver(simsettings.solver_name);
        global_settings->setLogSettings(simsettings.logSettings);
       // global_settings->setAlarmTime(simsettings.timeOut);

        global_settings->setAlarmTime(timeout);
        global_settings->setOutputPointType(simsettings.outputPointType);
        global_settings->setOutputFormat(simsettings.outputFormat);
        global_settings->setEmitResults(simsettings.emitResults);
        global_settings->setNonLinearSolverContinueOnError(simsettings.nonLinearSolverContinueOnError);
        global_settings->setSolverThreads(simsettings.solverThreads);
        /*shared_ptr<SimManager>*/ _simMgr = shared_ptr<SimManager>(new SimManager(mixedsystem, _config.get()));
         _simMgr->SetCheckTimeout(true);
        ISolverSettings* solver_settings = _config->getSolverSettings();
        solver_settings->setLowerLimit(simsettings.lower_limit);
        solver_settings->sethInit(simsettings.lower_limit);
        solver_settings->setUpperLimit(simsettings.upper_limit);
        solver_settings->setRTol(simsettings.tolerance);
        solver_settings->setATol(simsettings.tolerance);
        #ifdef RUNTIME_PROFILING
        if(MeasureTime::getInstance() != NULL)
        {
            MEASURETIME_END(measuredFunctionStartValues, measuredFunctionEndValues, (*measureTimeFunctionsArray)[0], simControllerInitializeHandler);
            measuredFunctionStartValues->reset();
            measuredFunctionEndValues->reset();
            MEASURETIME_START(measuredFunctionStartValues, simControllerSolveInitialSystemHandler, "SolveInitialSystem");
        }
        #endif

        _simMgr->initialize();

        #ifdef RUNTIME_PROFILING
        if(MeasureTime::getInstance() != NULL)
        {
            MEASURETIME_END(measuredFunctionStartValues, measuredFunctionEndValues, (*measureTimeFunctionsArray)[1], simControllerSolveInitialSystemHandler);
            MeasureTime::addResultContentBlock(mixedsystem->getModelName(),"simController",measureTimeFunctionsArray);
        }
        #endif



    }
    catch(ModelicaSimulationError & ex)
    {
        string error = add_error_info(string("Simulation failed for ") + simsettings.outputfile_name,ex.what(),ex.getErrorID());
        throw ModelicaSimulationError(SIMMANAGER, error, "", ex.isSuppressed());
    }
}

void SimController::Stop()
{
    if(_simMgr)
    _simMgr->stopSimulation();
}
/** @} */ // end of coreSimcontroller

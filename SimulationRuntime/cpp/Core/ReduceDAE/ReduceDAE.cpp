#include <Core/Modelica.h>
#include "com/modelicacompiler.h"
//#include <SimCoreFactory/OMCFactory/OMCFactory.h>
#include <Core/System/FactoryExport.h>
//cpp\SimCoreFactory\OMCFactory
#include <boost/lexical_cast.hpp>
//#include "../../Include/Core/System/IMixedSystem.h"
//#include "../../Include/Core/DataExchange/IHistory.h"
#include "../../Include/Core/ReduceDAE/IReduceDAE.h"
#include "../../Include/Core/ReduceDAE/IReduceDAESettings.h"
#include <Core/SimController/ISimController.h>
#include <Core/SimController/Configuration.h>
//#include <Core/System/FactoryExport.h>
//#include <Core/Utils/extension/logger.hpp>
//#include "Ranking.h"
//#include "Reduction.h"
#include "ReduceDAESettings.h"
//#include "simulation.h"
//#include "Configuration.h"
#include "LibrariesConfig.h"


#if defined(_MSC_VER) || defined(__MINGW32__)
#include <tchar.h>
int _tmain(int argc,const _TCHAR* argv[])
#else
int main(int argc, const char* argv[])
#endif
{
	IMixedSystem* system =NULL;
	try
	{

		string model;
		const char* file_path;
		//bool load_modelica_lib;

		//check for modelpath and model name
		if(argc==1)
		  throw std::runtime_error("No OMCData,model and file were passed");
		if(argc > 1)
		{
			model = argv[1];
			file_path = argv[2];
			//load_modelica_lib =false;
		}
		//check for load of modelica library for used Modelica model
		//if(argc > 3)
		//{
			//load_modelica_lib = boost::lexical_cast<bool>( argv[3]);
		//}

		//create ModelicaCompiler
		//ModelicaCompiler* compiler;
		//compiler =new ModelicaCompiler(model,file_name,load_modelica_lib);




		//generate code for  Modelica system with labels
		//if(reduce_settings.getReductionMethod() == IReduceDAESettings::CANCEL_TERMS)
			//std::vector<double> x;

			//compiler->generateLabeledSimCode("deletion",x);



		#ifdef RUNTIME_STATIC_LINKING

            shared_ptr<StaticOMCFactory>  _factory =  shared_ptr<StaticOMCFactory>(new StaticOMCFactory());

          #else


         shared_ptr<OMCFactory>  _factory =  shared_ptr<OMCFactory>(new OMCFactory());

          #endif //RUNTIME_STATIC_LINKING
		  std::map<std::string, std::string> opts;
		   std::pair<shared_ptr<ISimController>, SimSettings> simulation = _factory->createSimulation(argc, argv, opts);
		   //create Modelica system
          shared_ptr<ISimObjects> simObjects= simulation.first->getSimObjects();

		 weak_ptr<ISimData> simData = simObjects->LoadSimData(model);

         int numRealVars,numIntVars,numBoolVars,numStringVars,numPreVars,numStatevars,numStateVarIndex;
         /*for (int i = 1; i < argc; i++)
         {
         std::cout<<"argv"<<i<<" "<<std::stoi(argv[i+1])<<std::endl;}*/

         for (int i = 22; i < argc; i++) {

             if(strcmp(argv[i],("-nRV"))==0)
             {
             numRealVars=atoi(argv[i+1]);

             continue;
             }
             if(strcmp(argv[i],"-nIV")==0)
             {
             numIntVars=atoi(argv[i+1]);

             continue;
             }
             if(strcmp(argv[i],"-nBV")==0)
             {
             numBoolVars=atoi(argv[i+1]);

             continue;
             }
              if(strcmp(argv[i],"-nStrV")==0)
             {
             numStringVars=atoi(argv[i+1]);

             continue;
             }
             if(strcmp(argv[i],"-nPV")==0)
             {
             numPreVars=atoi(argv[i+1]);

             continue;
             }
             if(strcmp(argv[i],"-nStaV")==0)
             {
             numStatevars=atoi(argv[i+1]);

             continue;
             }
             if(strcmp(argv[i],"-nStaI")==0)
             {
             numStateVarIndex=atoi(argv[i+1]);

             continue;
             }
         }

          weak_ptr<ISimVars> simVars = simObjects->LoadSimVars(model,numRealVars,numIntVars,numBoolVars,numStringVars,numPreVars,numStatevars,numStateVarIndex);
         //weak_ptr<ISimVars> simVars = simObjects->LoadSimVars(model,43,1,0,0,44,2,0);
          weak_ptr<IMixedSystem> system = simulation.first->LoadSystem("OMCpp"+model+".dll",model);

          //simulation.first->Start(simulation.second, model);

		  //shared_ptr<IMixedSystem> system_reference=simulation.first->getSystem(model);
		 /* IReduceDAE* reduce_dae = dynamic_cast<IReduceDAE*>(system.lock().get());


		  if(reduce_dae==NULL)
		{
			throw std::runtime_error("Modelica System is not of type IReduceDAE!!!");
		}

			label_list_type labels = reduce_dae->getLabels();
		//number of labels
		int kDim = labels.size();


		label_type  label;
		//double time;
		int j=1;

		//loop over labels
        double mylabel01 = *(std::get<1>(labels[0]));
        double mylabel02 = *(std::get<2>(labels[0]));
        //unsigned int mylabel00= std::get<0>(labels[0]);
        //std::cout <<  "first value of label00: "<< mylabel00 << std::endl;
        std::cout <<  "first value of label01: "<< mylabel01 << std::endl;
        std::cout <<  "first value of label02: "<< mylabel02 << std::endl;
        *(std::get<1>(labels[0]))=0;
        *(std::get<2>(labels[0]))=1;
         mylabel01 = *(std::get<1>(labels[0]));
         mylabel02 = *(std::get<2>(labels[0]));
        // mylabel00= std::get<0>(labels[0]);
      // std::cout <<  "value of label00: "<< mylabel00 << std::endl;
       std::cout <<  "value of label01: "<< mylabel01 << std::endl;
       std::cout <<  "value of label02: "<< mylabel02 << std::endl;*/

        //std::get<2>($labels[0]) =1;
        simulation.first->TestLabeling(simulation.second, model);

        /*
		FOREACH(label, labels)
		{
			//index for current time entry
			//if(j<=kDim)

			//set label_1 to 0 and label_2 to 1
			*(get<1>(label))=0;
				//j=j+1;
		    *(get<2>(label))=1;

		}

		simulation.first->Start(simulation.second, model);*/


	}


	catch(std::runtime_error & ex)
	{
		string error = ex.what();
		cout<<error;
		return 1;
	}

	catch(std::exception & ex)
	{
		string error = ex.what();
		cout<<error;
		return 1;
	}

}

///*workaround bis settings serialisiert werden können*/
//	ublas::vector<double>::size_type i=0;
//	ublas::vector<double> max(argc-1);
//    while(*++argv)
//    {
//        try
//        {
//            max(i++)=lexical_cast<double>(*argv);
//        }
//        catch(bad_lexical_cast &)
//        {
//			throw std::runtime_error("Bad max error");
//        }
//    }
//	cout<<"Read max error: " << max;
//	reduce_settings.getMaxError() = max;

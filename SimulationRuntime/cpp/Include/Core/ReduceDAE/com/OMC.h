#pragma once
/**
 *  \file OMC.h
 *  \brief Interface for calls to OpenModelica Compiler(omc)
 */

#include "OMCAPI.h"

extern "C"
{
    /**
    Incomplete definition for OMCData
    */
    typedef struct OMCData data;


    /**
    *  \brief Allocates and initializes OpenModelica compiler(omc) instance
    *  \param [in] compiler name of c++ compiler (gcc for mingw compiler, msvc10 for Visual Studio 2010 compiler,msvc12 for Visual Studio 2012 compiler ...)
    *  \param [out] omcPtr  pointer to allocated omc instance
    *  \return returns a status flag
    */
   int OMC_DLL InitOMC(data** omcPtr,const char* compiler,const char* openModelicaHome);
   /**
    *  \brief returns the version of the OpenModelica compiler (omc) instance
    *  \param [in] omcPtr Pointer to omc instance
    *  \param [out] version string of omc instance
    *  \return returns a status flag
    */
   int OMC_DLL GetOMCVersion(data* omcPtr,char** result);

   /**
    *  \brief Free memory for OpenModelica compiler(omc) instance
    *  \param [in] omcPtr Pointer to omc instance
    */
   void OMC_DLL FreeOMC(data* omcPtr);

   /**
    *  \brief Return an error text for the last omc function call
    *  \param [in] omcPtr Pointer to omc instance
    *  \param [in] result includes an error text if an error occured else it is empty
    *  \return a status flag
    */
   int OMC_DLL GetError(data* omcPtr,char** result);


    /**
    *  \brief Loads Modelica library which is stored in OpenModelic home directory e.g LoadModel(omcData,"Modelica"); loads the latest MSL 3.2.1 library
    *  \param [in] omcPtr Pointer to omc instance
    *  \param [in] className Library name
    *  \param [in] version version number of library
    *  \return a status flag
    */
   int OMC_DLL LoadModel(data* omcPtr,const char* className);

   /**
    *  \brief Load a Modelica mo- file
    *
    *  \param [in] omcPtr Pointer to omc instance
    *  \param [in] fileName path to mo- file
    *  \return a status flag
    */
   int OMC_DLL LoadFile(data* omcPtr,const char* fileName);

   /**
    *  \brief Sends a command to OpenModelica compiler (omc)
    *  \param [in] omcPtr Pointer to omc instance
    *  \param [in] expression command expression e.g "loadFile(...)"
    *  \param [out] result Result of command execution
    *  \return  a status flag
    */
   int OMC_DLL SendCommand(data* omcPtr,const char* expression,char** result);


     /**
    *  \brief Configures OpenModelic compiler (omc) with a configuration string
    *  \param [in] omcPtr  Pointer to omc instance
    *  \param [in] expression configuration string e.g "+simCodeTarget=Cpp"
    *  \return a status flag
    */
   int OMC_DLL SetCommandLineOptions(data* omcPtr,const char* expression);

   /**
   *  \brief Setthe workingdirectoryto OpemModelica compiler (omc)
   *  \param [in] omcPtr  Pointer to omc instance
   *  \param [in] directory workingdirectory string e.g "C:\temp\"
   *  \return a status flag
   */
   int OMC_DLL SetWorkingDirectory(data* omcPtr, const char* directory, char** result);
}



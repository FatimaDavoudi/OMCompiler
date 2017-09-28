cmake_minimum_required (VERSION 2.6)

project(ReduceDAE)
#include_directories ("${MICO_INCLUDE_DIR}") Fatima
include_directories (${MICO_INCLUDE_DIR})
#add_executable(ReduceDAEInit com/ModelicaCompiler.cpp ReduceDAEInit.cpp ReduceDAESettings.cpp)
add_library(ReduceDAE STATIC com/ModelicaCompiler.cpp Simulation.cpp ReduceDAESettings.cpp Ranking.cpp Reduction.cpp)
#Simulation.cpp  ReduceDAE.cpp Configuration.cpp ReduceDAESettings.cpp)
#target_link_libraries( ReduceDAEInit ${Boost_LIBRARIES} ${OMCFactoryName} E:/ReduceDAE/OpenModelicaCompilerCWrapper/Binaries/lib/omc/libOMCDLL.dll.a)
# if(MSVC)
#	 target_link_libraries(ReduceDAE ${Boost_LIBRARIES} C:/OpenModelicaCompilerCWrapper/libOMCDLL.lib)
# else(MSVC)
    target_link_libraries(ReduceDAE ${Boost_LIBRARIES} C:/OpenModelicaCompilerCWrapper/Binaries/lib/omc/libOMCDLL.dll.a)
#endif(MSVC)

#target_link_libraries(ReduceDAE ${Boost_LIBRARIES})
# ${SystemName}
#install (TARGETS ReduceDAEInit DESTINATION bin)
#install (TARGETS ReduceDAE DESTINATION bin)
install(FILES "ReduceDAESettings.xml" DESTINATION share/omc/runtime/cpp/config)
install(FILES
  ${CMAKE_SOURCE_DIR}/Include/Core/ReduceDAE/IReduceDAE.h
  ${CMAKE_SOURCE_DIR}/Include/Core/ReduceDAE/IReduceDAESettings.h
  ${CMAKE_SOURCE_DIR}/Include/Core/ReduceDAE/ReduceDAESettings.h
  ${CMAKE_SOURCE_DIR}/Include/Core/ReduceDAE/Ranking.h
  ${CMAKE_SOURCE_DIR}/Include/Core/ReduceDAE/Reduction.h
  ${CMAKE_SOURCE_DIR}/Include/Core/ReduceDAE/Simulation.h
  ${CMAKE_SOURCE_DIR}/Include/Core/ReduceDAE/com/ModelicaCompiler.h
  DESTINATION include/omc/cpp/Core/ReduceDAE)






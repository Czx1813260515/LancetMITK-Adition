#Begin KUKA ROBOT
option(MITK_USE_KUKA_ROBOT "Enable support for KUKA Robot hardware" ON)

#only if MITK_USE_KUKA_ROBOT is enabled
if(MITK_USE_KUKA_ROBOT)
  if(CMAKE_BUILD_TYPE MATCHES "Debug")
    find_library(MITK_KUKA_ROBOT_LIB robotapid HINTS "${CMAKE_CURRENT_SOURCE_DIR}/robotapi/lib" DOC "Path which contains the robotapid.lib")
    STRING(REPLACE "robotapid.lib" "robotapid_Staubli.lib" MITK_KUKA_ROBOT_LIB "${MITK_KUKA_ROBOT_LIB}")
    message(WARNING "debug: ${MITK_KUKA_ROBOT_LIB}") 
  else()
    find_library(MITK_KUKA_ROBOT_LIB robotapi HINTS "${CMAKE_CURRENT_SOURCE_DIR}/robotapi/lib" DOC "Path which contains the robotapi.lib")
    STRING(REPLACE "robotapi.lib" "robotapi_Staubli.lib" MITK_KUKA_ROBOT_LIB "${MITK_KUKA_ROBOT_LIB}")
    message(WARNING "release: ${MITK_KUKA_ROBOT_LIB}")
  endif()
  
  
  
  find_path(MITK_KUKA_ROBOT_INCLUDE_DIR robotapi.h HINTS "${CMAKE_CURRENT_SOURCE_DIR}/robotapi/include" DOC  "Include directory of the robot api.")
ENDIF(MITK_USE_KUKA_ROBOT)

#End KUKA Hardware
set(AIMOOE_INCLUDE "")
set(AIMOOE_LIB_RELEASE "")
set(AIMOOE_DLLS_RELEASE "")

set(AIMOOE_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/AimPosDll/include")
file(GLOB AIMOOE_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/AimPosDll/Release/*.lib")

###############################################################################################

set(Diana_INCLUDE "")
set(Diana_LIB_RELEASE "")
set(Diana_DLLS_RELEASE "")

set(Diana_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/DianaApi/include")
file(GLOB Diana_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/DianaApi/Release/*.lib")
######################################################################################
set(Hans_INCLUDE "")
set(Hans_LIB_RELEASE "")
set(Hans_DLLS_RELEASE "")

set(Hans_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/HansRobotDll/include")
file(GLOB Hans_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/HansRobotDll/Release/*.lib")

mitk_create_module(
  INCLUDE_DIRS
     PUBLIC ${AIMOOE_INCLUDE}
     PUBLIC ${Diana_INCLUDE}
     PUBLIC ${Hans_INCLUDE}
  DEPENDS PUBLIC MitkCore 
  DEPENDS MitkLancetPrintDataHelper
  PACKAGE_DEPENDS VTK
  PACKAGE_DEPENDS Qt5|Core+Widgets
  ADDITIONAL_LIBS PUBLIC "${AIMOOE_LIB_RELEASE}"
  ADDITIONAL_LIBS PUBLIC "${Diana_LIB_RELEASE}"
  ADDITIONAL_LIBS PUBLIC "${Hans_LIB_RELEASE}"
)

add_subdirectory(cmdapps)

#option(BUILD_NewModuleMiniApps "Build commandline tools for the example module" OFF)

#if(BUILD_NewModuleMiniApps)

mitkFunctionCreateCommandLineApp(
  NAME FitCircleMiniApp
  DEPENDS MitkLancetAlgo
)

#endif()

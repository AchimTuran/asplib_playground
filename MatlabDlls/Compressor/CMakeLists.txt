# sets the solution name
project(CompressorDll)

set(MATLAB_FIND_DEBUG ON)
find_package(Matlab COMPONENTS MX_LIBRARY MAT_LIBRARY REQUIRED)

# set here further source and header files, which should be relative to this file
set(SOURCES CompressorDll.cpp
            Compressor.cpp)

set(HEADERS CompressorDll.h
            Compressor.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/../Common/typedefs.h)

# this will create filters in the visual studio solution
asplib_source_group("${SOURCES}")
asplib_source_group("${HEADERS}")

set(MATLAB_WKS $ENV{APPDATA})
string(REPLACE "\\" "/" MATLAB_WKS ${MATLAB_WKS})
set(MATLAB_WKS "${MATLAB_WKS}/../../documents/MATLAB")
message(STATUS "MATLAB_WKS:${MATLAB_WKS}")

# create the shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

target_compile_definitions(${PROJECT_NAME} PRIVATE ASPLIB_DLL_FUNCTION_EXPORT=1 _SCL_SECURE_NO_WARNINGS)

# add include directories
target_include_directories(${PROJECT_NAME} PRIVATE
                                           $<BUILD_INTERFACE:${Matlab_INCLUDE_DIRS}>
                                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../Common>)
                                           
target_compile_definitions(${PROJECT_NAME} PRIVATE )

# link against asplib SpectrumVisProcessor component
target_link_libraries(${PROJECT_NAME} asplib::Core ${Matlab_MEX_LIBRARY} ${Matlab_MX_LIBRARY} ${Matlab_MAT_LIBRARY})
# sets the exectuable output file name
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

# sets the visual studio solution folder (nested projects)
# NOTE: folders are separated with /
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "MatlabDlls")

# install all into MATLAB's user workspace
if(EXISTS ${MATLAB_WKS})
  install(TARGETS ${PROJECT_NAME} DESTINATION ${MATLAB_WKS}/asplib_playground/bin)
  install(FILES ${HEADERS} DESTINATION ${MATLAB_WKS}/asplib_playground/bin)
  install(DIRECTORY m-Files/ DESTINATION ${MATLAB_WKS}/asplib_playground FILES_MATCHING PATTERN "*.m")
  install(DIRECTORY m-Files/ DESTINATION ${MATLAB_WKS}/asplib_playground FILES_MATCHING PATTERN "*.wav")
else()
  message(WARNING "Skipped installation of MatlabDll ${PROJECT_NAME} because ${MATLAB_WKS} doesn't exist.")
endif()

matlab_get_version_from_release_name("R2017a" newAPIVersion)
if(${Matlab_VERSION_STRING} VERSION_GREATER ${newAPIVersion})
  message(WARNING "fallback to legacy API")
  target_compile_definitions(${PROJECT_NAME} PRIVATE MEX_DOUBLE_HANDLE)
endif()

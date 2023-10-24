# Define library. Only source files here!
set(PROJECT_NAME APIExampleCPPNetworkPlanning5G_TDD)
set(PROJECT_DESC "Example of computing a 5G-TDD network planning using the Altair WinProp CPP API")

cmake_minimum_required(VERSION 3.9)

project("${PROJECT_NAME}" 
	DESCRIPTION ${PROJECT_DESC}
	LANGUAGES CXX C)

add_library( Engine SHARED IMPORTED )
if(NOT WIN32)
set_target_properties( Engine PROPERTIES
	IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/../../bin/libEngine.so" 
	)
else(NOT WIN32)
set_target_properties( Engine PROPERTIES
	IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/../../export/winprop/64/Engine.dll" 
	IMPORTED_IMPLIB "${PROJECT_SOURCE_DIR}/../../export/winprop/64/Engine.lib" 
	)
endif(NOT WIN32)

add_library( EngineNet SHARED IMPORTED )
if(NOT WIN32)
set_target_properties( EngineNet PROPERTIES
	IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/../../bin/libEngineNet.so" 
	)
else(NOT WIN32)
set_target_properties( EngineNet PROPERTIES
	IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/../../export/winprop/64/EngineNet.dll" 
	IMPORTED_IMPLIB "${PROJECT_SOURCE_DIR}/../../export/winprop/64/EngineNet.lib" 
	)
endif(NOT WIN32)


add_executable(${PROJECT_NAME}
	"${PROJECT_SOURCE_DIR}/network_planning_5g_tdd.cpp"
	"${PROJECT_SOURCE_DIR}/network_planning_5g_tdd.h")

# Definition of Macros
target_compile_definitions(${PROJECT_NAME} PRIVATE 
	$<$<CONFIG:Debug>:_DEBUG>
	
	_CRT_SECURE_NO_WARNINGS
	
	_WINPROP_API_
	API_DATA_FOLDER=\"${PROJECT_SOURCE_DIR}/../../data/\"
	
	# Visual Studio character set "not set"
	_SBCS
	
	# library exports
	
	# library imports
	_DLL_ENGINE_IMPORT
	_DLL_WP_NET_IMPORT
)

if(NOT WIN32)
target_compile_definitions(${PROJECT_NAME} PRIVATE 
	__LINUX
	_NO_MFC
)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
	"${PROJECT_SOURCE_DIR}/../../source"
	"${PROJECT_SOURCE_DIR}/../../source/Public"
)

# If we have compiler requirements for this library, list them
# here
target_compile_features(${PROJECT_NAME}
    PUBLIC cxx_auto_type
    PRIVATE cxx_variadic_templates)

# Depend on a library that we defined in the top-level file
target_link_libraries(${PROJECT_NAME} PRIVATE
	Engine
	EngineNet)
	
set_target_properties(${PROJECT_NAME} 
	PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../../bin
	RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/../../bin
	RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/../../bin
	OUTPUT_NAME ${PROJECT_NAME}
	OUTPUT_NAME_DEBUG "${PROJECT_NAME}_debug"
)
# CMakeList.txt : CMake project for sim_ui, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

#REM Build for Visual Studio compiler. Run your copy of vcvars32.bat or vcvarsall.bat to setup command-line compiler.
#REM Important: to build on 32-bit systems, the DX12 backends needs '#define ImTextureID ImU64', so we pass it here.

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

project (sim_ui)

# imgui
set(IMGUI_PATH   "" CACHE FILEPATH "The IMGUI library")
message(STATUS "IMGUI library:     ${IMGUI_PATH}")

# implot
set(IMPLOT_PATH   "" CACHE FILEPATH "The IMPLOT library")
message(STATUS "IMPLOT library:     ${IMPLOT_PATH}")

# tinyxml2
set(TINYXML2_PATH   "" CACHE FILEPATH "The TINYXML2 library")
message(STATUS "TINYXML2 library:     ${TINYXML2_PATH}")  
 
# eigen
set(EIGEN_PATH   "" CACHE FILEPATH "The EIGEN library")
message(STATUS "EIGEN library:     ${EIGEN_PATH}")  
 

FILE(GLOB IMGUI_SRC ${IMGUI_PATH}/*.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	FILE(GLOB IMGUI_BACKENDS_SRC 
			${IMGUI_PATH}/backends/imgui_impl_win32.cpp 
			${IMGUI_PATH}/backends/imgui_impl_dx12.cpp)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	FILE(GLOB IMGUI_BACKENDS_SRC 
		${IMGUI_PATH}/backends/imgui_impl_glfw.cpp
		${IMGUI_PATH}/backends/imgui_impl_opengl3.cpp)
endif()   


FILE(GLOB IMPLOT_SRC 
			${IMPLOT_PATH}/implot.cpp
			${IMPLOT_PATH}/implot_items.cpp
			${IMPLOT_PATH}/implot_demo.cpp)

FILE(GLOB TINYXML2_SRC ${TINYXML2_PATH}/tinyxml2.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	FILE(GLOB SIM_UI_SRC sim_ui.cpp main_win.cpp)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	FILE(GLOB SIM_UI_SRC sim_ui.cpp main_linux.cpp)
endif()   


include_directories(${CMAKE_SOURCE_DIR}
					${IMGUI_PATH}
					${IMGUI_PATH}/backends 
					${IMPLOT_PATH}
					${TINYXML2_PATH}
					${EIGEN_PATH}/)
					
set(SIM_UI_HEADERS
    ${CMAKE_SOURCE_DIR}/sim_ui.h
    )	
	
set(SIM_UI_FILES
	${SIM_UI_HEADERS}
	${SIM_UI_SRC}							 				 
	${TINYXML2_PATH}
	${IMGUI_SRC}
	${IMGUI_BACKENDS_SRC}
	${IMPLOT_SRC}
	${TINYXML2_SRC}
	)
	
# Add source to this project's executable.
add_executable (${PROJECT_NAME} ${SIM_UI_FILES})

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	target_link_libraries(${PROJECT_NAME}
						d3d12.lib 
						d3dcompiler.lib 
						dxgi.lib
						)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    # linux: sudo apt install libglfw3-dev
    target_link_libraries(${PROJECT_NAME}
        GL
        glfw # use this lib name
        dl
    )
endif()
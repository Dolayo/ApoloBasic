cmake_minimum_required(VERSION 2.8)
#Provisional
PROJECT(ApoloLite)

	IF(WIN32)
		INCLUDE_DIRECTORIES(C:/mrcore/include) 
		LINK_DIRECTORIES(C:/mrcore/lib)
	ELSE(WIN32)
		INCLUDE_DIRECTORIES(/usr/local/mrcore/include)
		LINK_DIRECTORIES(/usr/local/mrcore/lib)
	ENDIF(WIN32)

FIND_PACKAGE(wxWidgets COMPONENTS gl net aui adv xml core base)
#find_package(OpenGL REQUIRED COMPONENTS OpenGL)

include_directories(${PROJECT_SOURCE_DIR}/include)

IF(wxWidgets_FOUND)
	INCLUDE(${wxWidgets_USE_FILE})
ELSE()
    message(FATAL_ERROR "YOU SHOULD HAVE LOCALLY INSTALLED THE WXWIDGETS PACKAGE")
ENDIF()

file(GLOB ALL_SRC "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB ALL_HDRS "${CMAKE_SOURCE_DIR}/include/*.h")
ADD_EXECUTABLE(ApoloLite  ${ALL_SRC} ${ALL_HDRS} ${RESOURCES})

#add_executable(ApoloLite src/main.cpp src/mainWindow.cpp src/canvas.cpp src/rdtstar.cpp)

IF(WIN32)
	SET_TARGET_PROPERTIES(ApoloLite PROPERTIES
		COMPILE_FLAGS "${wxWidgets_CXX_FLAGS} ${wxWidgets_DEFINITIONS}"
		LINK_FLAGS "/SUBSYSTEM:WINDOWS")


	SET_TARGET_PROPERTIES(ApoloLite PROPERTIES
		COMPILE_FLAGS "/D _CRT_SECURE_NO_WARNINGS"
		LINK_FLAGS "/SUBSYSTEM:WINDOWS")
ENDIF(WIN32)

IF(WIN32)	
	TARGET_LINK_LIBRARIES(ApoloLite debug mrcore_d ${wxWidgets_LIBRARIES})	
	TARGET_LINK_LIBRARIES(ApoloLite optimized mrcore ${wxWidgets_LIBRARIES})
ELSE(WIN32) #LINUX		
	TARGET_LINK_LIBRARIES(ApoloLite mrcore ${OPENGL_LIBRARIES} ${wxWidgets_LIBRARIES})
ENDIF(WIN32)
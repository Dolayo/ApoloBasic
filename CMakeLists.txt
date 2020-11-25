cmake_minimum_required(VERSION 2.8)

PROJECT(ApoloBasic)

IF(WIN32)
		INCLUDE_DIRECTORIES(C:/mrcore/include) 
		LINK_DIRECTORIES(C:/mrcore/lib)
	ELSE(WIN32)
		INCLUDE_DIRECTORIES(/usr/local/mrcore/include)
		LINK_DIRECTORIES(/usr/local/mrcore/lib)
	ENDIF(WIN32)

FIND_PACKAGE(wxWidgets COMPONENTS gl net aui adv xml core base)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)

include_directories(${PROJECT_SOURCE_DIR}/include)

IF(wxWidgets_FOUND)
	INCLUDE(${wxWidgets_USE_FILE})
ELSE()
    message(FATAL_ERROR "YOU SHOULD HAVE LOCALLY INSTALLED THE WXWIDGETS PACKAGE")
ENDIF()

add_executable(myapp src/pruebawx.cpp)
target_link_libraries(myapp mrcore ${OPENGL_LIBRARIES} ${wxWidgets_LIBRARIES})

add_executable(ApoloBasic src/main.cpp src/mainWindow.cpp src/canvas.cpp)
target_link_libraries(ApoloBasic mrcore ${OPENGL_LIBRARIES} ${wxWidgets_LIBRARIES})

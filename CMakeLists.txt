cmake_minimum_required(VERSION 2.6)

#--------------------------------------------
#
# DEFINITION DU PROJET
#
#--------------------------------------------

#Projet
project(GAITAN)

#Version
set(GAITAN_VERSION_MAJOR 1)
set(GAITAN_VERSION_MINOR 0)

MESSAGE(STATUS "Le logiciel est ici : ${GAITAN_BINARY_DIR}")

# The location in which to install libraries.
SET(LIBRARY_OUTPUT_PATH ${GAITAN_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})
# The location in which to install GAITAN binaries.
SET(BINARY_OUTPUT_PATH ${GAITAN_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})
# The location in which to install GAITAN exe.
set(EXECUTABLE_OUTPUT_PATH ${GAITAN_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})


SET(GAITAN_INTERN_LINK_DIR ${GAITAN_SOURCE_DIR}/lib)
# Give a name to the intern lib
SET(GAITAN_INTERN_LIBS gaitan)
# the include directory we depend on
SET(GAITAN_INTERN_INCLUDE_DIR ${GAITAN_SOURCE_DIR}/include)

# the third party include directories we depend on
SET(GAITAN_EXTERN_INCLUDE_DIR "")
# the third party libraries we depend on
SET(GAITAN_EXTERN_LIBS "")
# path to the lib
SET(GAITAN_EXTERN_LINK_DIR "")

#--------------------------------------------
#
# LIBRARY THIRD PART
#
#--------------------------------------------

#Inclusion de boost
#set(Boost_DIR /usr/)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREAD OFF)
include(FindBoost)
find_package(Boost
	1.36.0
	REQUIRED 
	filesystem 
	system 
	iostreams
)
if(NOT Boost_FOUND)
	Message(STATUS "Boost is not Found !")
endif()
if(Boost_FOUND)
	Message(STATUS "Boost is Found !")
	#include_directories(${Boost_INCLUDE_DIRS})
	#link_directories(${Boost_LIBRARY_DIRS})

	  LIST(APPEND GAITAN_EXTERN_INCLUDE_DIR  ${Boost_INCLUDE_DIR})
   	LIST(APPEND GAITAN_EXTERN_LINK_DIR ${Boost_LIBRARY_DIRS}) 
   	LIST(APPEND GAITAN_EXTERN_LIBS  ${Boost_LIBRARIES})
	Message (STATUS ${GAITAN_EXTERN_LIBS})

endif()

#Ajout des lib de base libfreenect
LIST(APPEND GAITAN_EXTERN_INCLUDE_DIR /usr/local/include/libfreenect)
LIST(APPEND GAITAN_EXTERN_INCLUDE_DIR /usr/local/include/)
LIST(APPEND GAITAN_EXTERN_LINK_DIR /usr/local/lib64) 
LIST(APPEND GAITAN_EXTERN_LIBS  freenect)
LIST(APPEND GAITAN_EXTERN_LIBS  freenect_sync)
LIST(APPEND GAITAN_EXTERN_LIBS  GL)
LIST(APPEND GAITAN_EXTERN_LIBS  GLU)
LIST(APPEND GAITAN_EXTERN_LIBS  glut)


#Ajout de Visp
find_package(VISP REQUIRED)
if(VISP_FOUND)
	include(${VISP_USE_FILE})
endif(VISP_FOUND)


find_package(PCL 1.2 REQUIRED)
LIST(APPEND GAITAN_EXTERN_INCLUDE_DIR ${PCL_INCLUDE_DIRS})
LIST(APPEND GAITAN_EXTERN_LINK_DIR ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
MESSAGE(STATUS "library de la pcl ${PCL_LIBRARY_DIRS}")
#add_executable (cloud_viewer cloud_viewer.cpp)
#target_link_libraries (cloud_viewer ${PCL_LIBRARIES})
LIST(APPEND GAITAN_EXTERN_LIBS ${PCL_LIBRARIES})

#Inclusion de gnupot pour C++
#LIST(APPEND GAITAN_EXTERN_INCLUDE_DIR ~/projects/gnuplot-iostream)

#Ajout de open Ni
SET(openni_INCLUDE_DIR ~/projects/openni/Include)
LIST(APPEND GAITAN_EXTERN_INCLUDE_DIR ${openni_INCLUDE_DIR})

SET(openni_LINKS_DIR "")
LIST(APPEND openni_LINKS_DIR ${GAITAN_BINARY_DIR}/bin)
LIST(APPEND openni_LINKS_DIR ./)
LIST(APPEND GAITAN_EXTERN_LINK_DIR ${openni_LINKS_DIR})

LIST(APPEND GAITAN_EXTERN_LIBS  OpenNI2)

#Ajout de la lib eigen
LIST(APPEND GAITAN_EXTERN_INCLUDE_DIR /usr/local/include/eigen3)

#Ajout de la lib phidegt
LIST(APPEND GAITAN_EXTERN_LIBS libphidget21)

#----------------------------------------------------------------------
# Concatenate intern and third party include dirs
#----------------------------------------------------------------------
INCLUDE_DIRECTORIES(${GAITAN_INTERN_INCLUDE_DIR} ${GAITAN_EXTERN_INCLUDE_DIR})
#MESSAGE (STATUS "Include ${GAITAN_EXTERN_INCLUDE_DIR}")

#----------------------------------------------------------------------
# Add library path
#----------------------------------------------------------------------
LINK_DIRECTORIES(${GAITAN_INTERN_LINK_DIR} ${GAITAN_EXTERN_LINK_DIR})
#MESSAGE(STATUS " Link Libs ${GAITAN_EXTERN_LINK_DIR}")

#----------------------------------------------------------------------
# Link Libraries
# ----------------------------------------------------------------------
LINK_LIBRARIES(${GAITAN_EXTERN_LIBS} ${GAITAN_INTERN_LIBS})
#Message(STATUS "Libs ${GAITAN_EXTERN_LIBS}")
#----------------------------------------------------------------------
# CREATE INTERN LIBRARY
# ----------------------------------------------------------------------
file(GLOB_RECURSE
source_files
src/*
include/*)

#creation de la librairie associée au projet
add_library(gaitan
	#SHARED #library dynamique
	STATIC #library statique
	${source_files}
)

#add_subdirectory(src)
add_subdirectory(test)

#add_subdirectory(include)

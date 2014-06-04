#--------------------------------------------
#
# LISTE DES EXECUTABLES
#
#--------------------------------------------

SET (SOURCE
#main.cpp
#testPlotEncoder.cpp
#testGlut.cpp
#testPlaneFitting.cpp
#testEigen.cpp
testTable.cpp
testFoot.cpp
)

# rule for binary build
FOREACH(source ${SOURCE})
  # Compute the name of the binary to create
  GET_FILENAME_COMPONENT(binary ${source} NAME_WE)

  # From source compile the binary and add link rules
  ADD_EXECUTABLE(${binary} ${source})
  TARGET_LINK_LIBRARIES(${binary} 
    ${GAITAN_INTERN_LIBS} ${GAITAN_EXTERN_LIBS}
    )
  # Add test
 ADD_TEST(${binary} ${binary})

ENDFOREACH(source)

add_subdirectory(Sensor) 
add_subdirectory(ImagesIO) 
#add_subdirectory(Polytope) 
add_subdirectory(Geometry)
add_subdirectory(Application)
add_subdirectory(Phidget)

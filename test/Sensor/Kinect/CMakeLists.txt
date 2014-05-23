#--------------------------------------------
#
# LISTE DES EXECUTABLES
#
#--------------------------------------------

SET (SOURCE
#testVispKinect.cpp
#testOpenNi.cpp
#testFreenect.cpp
#testKinectDisplay.cpp
#testKinect.cpp
#testKinectAcquire.cpp
#testKinectRead.cpp
#testKinectConvertToEigenMatrix.cpp
#testKinectDetectGround.cpp
#testKinectConvert.cpp
#testKinectBoxSelectionMultiImage.cpp # set box and use it to split the ptcloud
#testKinectCalibrationMultiImage.cpp #detect a plane and use it to split the ptcloud
#testKinectCalibrationMultiImage-v2.cpp #using directly the param of kinect 
testKinectBoxOnFeet.cpp
testKinectChangeTimeFile.cpp
testKinectVisualizeFeet.cpp
testKinectVisualizeFeetInWorld.cpp
testKinectLimitFov.cpp
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

#add_subdirectory(test)

cmake_minimum_required(VERSION 2.8)
project(amrModule)
find_package(qibuild)

set(INCLUDE_DIR include)
set(SRC_DIR src)

include_directories(${INCLUDE_DIR})

set(_srcs
  ${INCLUDE_DIR}/potentialField.h
  ${INCLUDE_DIR}/controller.h
  ${INCLUDE_DIR}/amrModule.h
  ${SRC_DIR}/potentialField.cpp
  ${SRC_DIR}/amrModule.cpp
  ${SRC_DIR}/amrModuleMain.cpp  
)

qi_create_bin(amrModule ${_srcs})

qi_use_lib(amrModule ALCOMMON ALVISION OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_VIDEO OPENCV2_IMGPROC OPENCV2_CALIB3D)

#cmake_policy(VERSION 1.6)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_BACKWARDS_COMPATIBILITY 1.6)

project(r2module)
find_package(qibuild)

aux_source_directory(. SRC_LIST)

set(src_path ./src)
set(include_path ./include)

include_directories(/usr/local/include/eigen3/ ${include_path})

set(_srcs
  ${include_path}/configReader.h
  ${include_path}/libmath/transform.h
  ${include_path}/libmath/kinChain.h
  ${include_path}/libmath/Rutils.h
  ${include_path}/task.h
  ${include_path}/r2Module.h

  ${src_path}/configReader.cpp
  ${src_path}/libmath/transform.cpp
  ${src_path}/libmath/kinChain.cpp
  ${src_path}/libmath/Rutils.cpp
  ${src_path}/task.cpp
  ${src_path}/r2ModuleMain.cpp

  ${src_path}/r2Module.cpp
)


# Add soth libraries
add_subdirectory(soth-master)

set(executable_name r2Module)
qi_create_bin(r2Module ${_srcs})

add_dependencies(${executable_name} ${PROJECT_NAME})
target_link_libraries(${executable_name} ${PROJECT_NAME})

qi_use_lib(r2Module ALCOMMON ALVISION OPENCV2_CORE OPENCV2_HIGHGUI)


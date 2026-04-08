cmake_minimum_required(VERSION 3.8)
project(daru_mujoco)

if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

function(find_vendored_or_system_library out_var)
  set(options)
  set(one_value_args ROOT HEADER LIBNAME)
  set(multi_value_args LIB_CANDIDATES EXTRA_INCLUDE_SUFFIXES EXTRA_LIB_SUFFIXES)
  cmake_parse_arguments(FVOSL "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  if(NOT FVOSL_ROOT)
    message(FATAL_ERROR "find_vendored_or_system_library requires ROOT")
  endif()

  set(_include_candidates
    ${FVOSL_ROOT}
    ${FVOSL_ROOT}/include
  )
  foreach(_suffix IN LISTS FVOSL_EXTRA_INCLUDE_SUFFIXES)
    list(APPEND _include_candidates ${FVOSL_ROOT}/${_suffix})
  endforeach()

  set(_lib_candidates
    ${FVOSL_ROOT}
    ${FVOSL_ROOT}/lib
    ${FVOSL_ROOT}/lib64
  )
  foreach(_suffix IN LISTS FVOSL_EXTRA_LIB_SUFFIXES)
    list(APPEND _lib_candidates ${FVOSL_ROOT}/${_suffix})
  endforeach()

  set(_vendored_header_dir "")
  if(FVOSL_HEADER)
    find_path(_vendored_header_dir
      NAMES ${FVOSL_HEADER}
      PATHS ${_include_candidates}
      NO_DEFAULT_PATH
    )
  endif()

  set(_vendored_library "")
  if(FVOSL_LIB_CANDIDATES)
    find_library(_vendored_library
      NAMES ${FVOSL_LIB_CANDIDATES}
      PATHS ${_lib_candidates}
      NO_DEFAULT_PATH
    )
  endif()

  if(_vendored_library)
    set(${out_var}_FOUND TRUE PARENT_SCOPE)
    set(${out_var}_INCLUDE_DIR ${_vendored_header_dir} PARENT_SCOPE)
    set(${out_var}_LIBRARY ${_vendored_library} PARENT_SCOPE)
  else()
    set(${out_var}_FOUND FALSE PARENT_SCOPE)
  endif()
endfunction()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(CURL REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(TIFF REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
pkg_check_modules(CAIRO REQUIRED cairo)

set(VENDORED_MUJOCO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco-3.4.0)
set(VENDORED_MUJOCO_INCLUDE ${VENDORED_MUJOCO_ROOT}/include)
set(VENDORED_MUJOCO_LIB_SO ${VENDORED_MUJOCO_ROOT}/lib/libmujoco.so.3.4.0)
set(VENDORED_MUJOCO_LIB_LINK ${VENDORED_MUJOCO_ROOT}/lib/libmujoco.so)
set(VENDORED_RBDL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rbdl)
set(VENDORED_GLFW_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw)

if(EXISTS ${VENDORED_MUJOCO_INCLUDE}/mujoco/mujoco.h AND EXISTS ${VENDORED_MUJOCO_LIB_SO})
  set(MUJOCO_INCLUDE_DIR ${VENDORED_MUJOCO_INCLUDE})
  set(MUJOCO_LIBRARY ${VENDORED_MUJOCO_LIB_SO})
else()
  find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h)
  find_library(MUJOCO_LIBRARY mujoco)
endif()

set(VENDORED_GLFW_INCLUDE ${VENDORED_GLFW_ROOT}/include)
set(VENDORED_GLFW_LIB ${VENDORED_GLFW_ROOT}/lib/libglfw.so)
set(VENDORED_GLFW_LIB_SONAME ${VENDORED_GLFW_ROOT}/lib/libglfw.so.3)
set(VENDORED_GLFW_LIB_REAL ${VENDORED_GLFW_ROOT}/lib/libglfw.so.3.3)
if(EXISTS ${VENDORED_GLFW_INCLUDE}/GLFW/glfw3.h AND EXISTS ${VENDORED_GLFW_LIB})
  set(VENDORED_GLFW_FOUND TRUE)
  set(GLFW_INCLUDE_DIR ${VENDORED_GLFW_INCLUDE})
  if(EXISTS ${VENDORED_GLFW_LIB_SONAME})
    set(GLFW_LIBRARY ${VENDORED_GLFW_LIB_SONAME})
  else()
    set(GLFW_LIBRARY ${VENDORED_GLFW_LIB})
  endif()
else()
  find_path(GLFW_INCLUDE_DIR GLFW/glfw3.h)
  find_library(GLFW_LIBRARY glfw)
endif()

set(VENDORED_RBDL_INCLUDE ${VENDORED_RBDL_ROOT}/include)
set(VENDORED_RBDL_LIB ${VENDORED_RBDL_ROOT}/lib/librbdl.so)
set(VENDORED_RBDL_LIB_REAL ${VENDORED_RBDL_ROOT}/lib/librbdl.so.3.3.1)
set(VENDORED_RBDL_URDF_LIB ${VENDORED_RBDL_ROOT}/lib/librbdl_urdfreader.so)
set(VENDORED_RBDL_URDF_LIB_REAL ${VENDORED_RBDL_ROOT}/lib/librbdl_urdfreader.so.3.3.1)
if(EXISTS ${VENDORED_RBDL_INCLUDE}/rbdl/rbdl.h AND EXISTS ${VENDORED_RBDL_LIB} AND EXISTS ${VENDORED_RBDL_URDF_LIB})
  set(VENDORED_RBDL_FOUND TRUE)
  set(VENDORED_RBDL_URDF_FOUND TRUE)
  set(RBDL_INCLUDE_DIR ${VENDORED_RBDL_INCLUDE})
  set(RBDL_INCLUDE_DIRS ${VENDORED_RBDL_INCLUDE})
  if(EXISTS ${VENDORED_RBDL_LIB_REAL})
    set(RBDL_LIBRARY ${VENDORED_RBDL_LIB_REAL})
  else()
    set(RBDL_LIBRARY ${VENDORED_RBDL_LIB})
  endif()
  if(EXISTS ${VENDORED_RBDL_URDF_LIB_REAL})
    set(RBDL_URDFREADER_LIBRARY ${VENDORED_RBDL_URDF_LIB_REAL})
  else()
    set(RBDL_URDFREADER_LIBRARY ${VENDORED_RBDL_URDF_LIB})
  endif()
else()
  find_package(RBDL REQUIRED)
endif()

if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY AND GLFW_LIBRARY AND RBDL_LIBRARY AND RBDL_URDFREADER_LIBRARY)
  add_executable(daru_v4_mujoco_teleop_sim
    src/package_paths.cpp
    src/teleop/daru_v4_mujoco_teleop_main.cpp
    src/teleop/daru_v4_mujoco_teleop_controller.cpp
    src/teleop/daru_v4_mujoco_teleop_gui.cpp
    src/teleop/daru_v4_mujoco_teleop_ros.cpp
    src/daru_v4_rbdl.cpp
  )

  add_executable(daru_v4_mujoco_gr00t_infer_sim
    src/package_paths.cpp
    src/gr00t_infer/daru_v4_mujoco_gr00t_infer_main.cpp
    src/gr00t_infer/daru_v4_mujoco_gr00t_infer_controller.cpp
    src/gr00t_infer/daru_v4_mujoco_gr00t_infer_gui.cpp
    src/gr00t_infer/daru_v4_mujoco_gr00t_infer_ros.cpp
    src/daru_v4_rbdl.cpp
  )

  target_include_directories(daru_v4_mujoco_teleop_sim PRIVATE
    include
    ${MUJOCO_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Eigen3_INCLUDE_DIRS}
    ${RBDL_INCLUDE_DIR}
    ${RBDL_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  target_include_directories(daru_v4_mujoco_gr00t_infer_sim PRIVATE
    include
    ${MUJOCO_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Eigen3_INCLUDE_DIRS}
    ${RBDL_INCLUDE_DIR}
    ${RBDL_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  target_link_directories(daru_v4_mujoco_teleop_sim PRIVATE
    ${CAIRO_LIBRARY_DIRS}
  )

  target_link_directories(daru_v4_mujoco_gr00t_infer_sim PRIVATE
    ${CAIRO_LIBRARY_DIRS}
  )

  target_link_libraries(daru_v4_mujoco_teleop_sim
    ${MUJOCO_LIBRARY}
    ${GLFW_LIBRARY}
    OpenGL::GL
    ${CAIRO_LIBRARIES}
    ${OpenCV_LIBS}
    ${Eigen3_LIBRARIES}
    ${RBDL_LIBRARY}
    ${RBDL_URDFREADER_LIBRARY}
    TIFF::TIFF
    CURL::libcurl
  )

  target_link_libraries(daru_v4_mujoco_gr00t_infer_sim
    ${MUJOCO_LIBRARY}
    ${GLFW_LIBRARY}
    OpenGL::GL
    ${CAIRO_LIBRARIES}
    ${OpenCV_LIBS}
    ${Eigen3_LIBRARIES}
    ${RBDL_LIBRARY}
    ${RBDL_URDFREADER_LIBRARY}
    TIFF::TIFF
    CURL::libcurl
  )

  ament_target_dependencies(daru_v4_mujoco_teleop_sim
    ament_index_cpp
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    visualization_msgs
    image_transport
  )

  ament_target_dependencies(daru_v4_mujoco_gr00t_infer_sim
    ament_index_cpp
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    visualization_msgs
    image_transport
  )

  set_target_properties(daru_v4_mujoco_teleop_sim PROPERTIES
    INSTALL_RPATH "$ORIGIN"
  )

  set_target_properties(daru_v4_mujoco_gr00t_infer_sim PROPERTIES
    INSTALL_RPATH "$ORIGIN"
  )

  install(TARGETS daru_v4_mujoco_teleop_sim daru_v4_mujoco_gr00t_infer_sim
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  if(EXISTS ${VENDORED_MUJOCO_LIB_SO})
    install(FILES ${VENDORED_MUJOCO_LIB_SO}
      DESTINATION lib/${PROJECT_NAME}
    )
    if(EXISTS ${VENDORED_MUJOCO_LIB_LINK})
      install(FILES ${VENDORED_MUJOCO_LIB_LINK}
        DESTINATION lib/${PROJECT_NAME}
      )
    endif()
  endif()

  if(VENDORED_GLFW_FOUND)
    install(FILES ${VENDORED_GLFW_LIB}
      DESTINATION lib/${PROJECT_NAME}
    )
    if(EXISTS ${VENDORED_GLFW_LIB_SONAME})
      install(FILES ${VENDORED_GLFW_LIB_SONAME}
        DESTINATION lib/${PROJECT_NAME}
      )
    endif()
    if(EXISTS ${VENDORED_GLFW_LIB_REAL})
      install(FILES ${VENDORED_GLFW_LIB_REAL}
        DESTINATION lib/${PROJECT_NAME}
      )
    endif()
  endif()

  if(VENDORED_RBDL_FOUND)
    install(FILES ${VENDORED_RBDL_LIB}
      DESTINATION lib/${PROJECT_NAME}
    )
    if(EXISTS ${VENDORED_RBDL_LIB_REAL})
      install(FILES ${VENDORED_RBDL_LIB_REAL}
        DESTINATION lib/${PROJECT_NAME}
      )
    endif()
  endif()

  if(VENDORED_RBDL_URDF_FOUND)
    install(FILES ${VENDORED_RBDL_URDF_LIB}
      DESTINATION lib/${PROJECT_NAME}
    )
    if(EXISTS ${VENDORED_RBDL_URDF_LIB_REAL})
      install(FILES ${VENDORED_RBDL_URDF_LIB_REAL}
        DESTINATION lib/${PROJECT_NAME}
      )
    endif()
  endif()
else()
  message(WARNING "MuJoCo/GLFW/RBDL dependency not found. Skipping daru_v4_mujoco teleop/gr00t_infer targets.")
endif()

install(DIRECTORY include/
  DESTINATION include
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/aaa.xml)
  install(FILES aaa.xml
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/meshes)
  install(DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY third_party/mujoco-3.4.0/include
  DESTINATION share/${PROJECT_NAME}/third_party/mujoco-3.4.0
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.10)
project(mujoco-wasm)

set(SOURCE_FILES src/main.genned.cpp)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/dist)

# Generate compile_commands.json for LSP
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Define Emscripten flags as a single string with proper spacing
if (EMSCRIPTEN)
  set(EMCC_LINKER_FLAGS "-s WASM=1 -s USE_WEBGL2=1 -s ASSERTIONS=1 --bind -s ALLOW_MEMORY_GROWTH=1 -s EXPORT_ES6=1 -s MODULARIZE=1 -s FORCE_FILESYSTEM=1 -s EXPORTED_RUNTIME_METHODS=['FS','MEMFS','ccall','cwrap'] -s EXPORT_NAME=load_mujoco -s EXCEPTION_CATCHING_ALLOWED=['load_from_xml']")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EMCC_LINKER_FLAGS}")
endif()

set(MAIN_PATH "${PROJECT_SOURCE_DIR}/..")

if (EMSCRIPTEN)
  set(MUJOCO_FRAMEWORK_PATH "${MAIN_PATH}/mujoco-wasm/build_wasm")
  message(STATUS "MUJOCO_FRAMEWORK_PATH (Emscripten): ${MUJOCO_FRAMEWORK_PATH}")
  set(MUJOCO_INCLUDE_DIR "${MUJOCO_FRAMEWORK_PATH}/include")
  
  # For Emscripten build, we'll use individual component libraries
  set(MUJOCO_LIBRARIES 
    "${MUJOCO_FRAMEWORK_PATH}/lib/libccd.a"
    "${MUJOCO_FRAMEWORK_PATH}/lib/libelasticity.a"
    "${MUJOCO_FRAMEWORK_PATH}/lib/liblodepng.a"
    "${MUJOCO_FRAMEWORK_PATH}/lib/libmujoco.a"
    "${MUJOCO_FRAMEWORK_PATH}/lib/libtinyxml2.a"
    "${MUJOCO_FRAMEWORK_PATH}/lib/libqhullstatic_r.a"
    "${MUJOCO_FRAMEWORK_PATH}/lib/libactuator.a"
    "${MUJOCO_FRAMEWORK_PATH}/lib/libsdf.a"
    "${MUJOCO_FRAMEWORK_PATH}/lib/libsensor.a"
    "${MUJOCO_FRAMEWORK_PATH}/lib/libspdlog.a"
  )
  
  add_executable(mujoco-wasm ${SOURCE_FILES})
  target_include_directories(mujoco-wasm PRIVATE ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(mujoco-wasm ${MUJOCO_LIBRARIES})
  
else()
  # Apple build
  set(MUJOCO_FRAMEWORK_PATH "${MAIN_PATH}/mujoco-wasm/build_apple/lib/mujoco.framework")
  message(STATUS "MUJOCO_FRAMEWORK_PATH (Apple): ${MUJOCO_FRAMEWORK_PATH}")
  set(MUJOCO_INCLUDE_DIR "${MUJOCO_FRAMEWORK_PATH}")
  set(MUJOCO_LIB "${MUJOCO_FRAMEWORK_PATH}/Versions/Current/libmujoco.dylib")
  
  set(GLFW_INCLUDE_DIR "${MAIN_PATH}/glfw-3.4/include")
  set(GLFW_LIB "${MAIN_PATH}/glfw-3.4/build/src/libglfw3.a")
  
  add_executable(mujoco-apple ${SOURCE_FILES})
  target_include_directories(mujoco-apple PRIVATE 
    ${MUJOCO_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIR}
  )
  target_link_libraries(mujoco-apple ${MUJOCO_LIB} ${GLFW_LIB})
  
  # Add framework linking if needed for macOS
  set_target_properties(mujoco-apple PROPERTIES
    LINK_FLAGS "-framework Cocoa -framework OpenGL -framework IOKit"
  )
endif()


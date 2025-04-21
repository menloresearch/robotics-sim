cmake_minimum_required(VERSION 3.10)
project(mujoco-wasm)

set(SOURCE_FILES src/main.cpp)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/dist)

# Generate compile_commands.json for LSP
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Define Emscripten flags as a single string with proper spacing
if (EMSCRIPTEN)
  # set(EMCC_LINKER_FLAGS "-s WASM=1 -s USE_WEBGL2=1 -s ASSERTIONS=1 --bind -s ALLOW_MEMORY_GROWTH=1 -s EXPORT_ES6=1 -s FORCE_FILESYSTEM=1 -s EXIT_RUNTIME=1 -s EXPORTED_RUNTIME_METHODS=['FS','MEMFS','ccall','cwrap'] -s EXPORTED_FUNCTIONS=['_main']")
  set(EMCC_LINKER_FLAGS "-s EXIT_RUNTIME=1 -s WASM=1")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EMCC_LINKER_FLAGS}")
endif()

set(MAIN_PATH "${PROJECT_SOURCE_DIR}/..")

if (EMSCRIPTEN)
  set(MUJOCO_FRAMEWORK_PATH "${MAIN_PATH}/mujoco-wasm/build_wasm")
  set(MUJOCO_LIB "${MUJOCO_FRAMEWORK_PATH}/lib")
  set(MUJOCO_INCLUDE_DIR "${MUJOCO_FRAMEWORK_PATH}/include")
else()
  set(MUJOCO_FRAMEWORK_PATH "${MAIN_PATH}/mujoco-wasm/build_apple/lib/mujoco.framework")
  set(MUJOCO_LIB "${MUJOCO_FRAMEWORK_PATH}/Versions/Current/libmujoco.dylib")
  set(MUJOCO_INCLUDE_DIR "${MUJOCO_FRAMEWORK_PATH}/Headers")

  set(GLFW_LIB "${MAIN_PATH}/glfw-3.4/build/src/libglfw3.a")
  set(GLFW_INCLUDE_DIR "${MAIN_PATH}/glfw-3.4/include")
endif()

include_directories(${MUJOCO_INCLUDE_DIR})

link_directories(${MUJOCO_LIB})

if (EMSCRIPTEN)
  add_executable(mujoco-wasm ${SOURCE_FILES})
  target_link_libraries(mujoco-wasm ccd elasticity lodepng mujoco tinyxml2 qhullstatic_r actuator sdf sensor spdlog)
else()
  add_executable(mujoco-apple ${SOURCE_FILES})
  target_link_libraries(mujoco-apple mujoco glfw3)
endif()

# On macOS, you need additional frameworks
if(APPLE)
    target_link_libraries(mujoco-apple "-framework Cocoa" "-framework IOKit" "-framework CoreFoundation" "-framework CoreVideo" "-framework OpenGL")
endif()


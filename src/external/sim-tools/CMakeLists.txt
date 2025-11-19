
cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
project(sim-tools)

find_package(assimp REQUIRED)
find_package(Freetype REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

#This must be specified according to the nvidia card being used. GeForce 940MX is 5.0 (automatize this)
set(CMAKE_CUDA_ARCHITECTURES 50) 

 #removes annoying warnings produced by eigen when compiling cuda. Perhaps use with caution in case the warnings are actually problematic
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcudafe --diag_suppress=20011 -Xcudafe --diag_suppress=20012 -Xcudafe --diag_suppress=20014 -Xcudafe --diag_suppress=20044 --expt-relaxed-constexpr")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS}" PARENT_SCOPE) #Propagate this option to the project using sim-tools

set(SRC
    src/yaml-parser.cpp   
    src/timer.cpp 
    src/gui/graphics/camera.cpp
    src/gui/graphics/graphics-model.cpp
    src/gui/graphics/graphics-mesh.cpp
    src/gui/graphics/shader.cpp
    src/gui/graphics/triad.cpp
    src/gui/graphics/text-renderer.cpp
    src/gui/graphics/colorbar.cpp
    src/gui/graphics/arrow-renderer.cpp
    src/gui/graphics/set-vertex-attrib-pointer.cpp
    vendor/stb/stb_image_impl.cpp

    #ImGUI files:
    vendor/imgui/implot_src/implot.cpp
    vendor/imgui/implot_src/implot_items.cpp
    vendor/imgui/imgui_src/imgui.cpp
    vendor/imgui/imgui_src/imgui_draw.cpp
    vendor/imgui/imgui_src/imgui_impl_glfw.cpp
    vendor/imgui/imgui_src/imgui_impl_opengl3.cpp
    vendor/imgui/imgui_src/imgui_tables.cpp
    vendor/imgui/imgui_src/imgui_widgets.cpp
    vendor/imgui/implot_src/implot.h
)

set(SRC_CUDA
    src/arena.cu
)


#build libraries if not already built
execute_process(COMMAND ./build-yaml-cpp.sh 
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})  

# execute_process(COMMAND ./build-assimp.sh 
#                 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})  


# Create the base library
add_library(${PROJECT_NAME} STATIC ${SRC})

if(CUDA_ENABLED)
    enable_language(CUDA)
    add_compile_definitions(CUDA_BUILD) ##add this globally
    add_library(${PROJECT_NAME}-cuda STATIC ${SRC_CUDA})
    target_compile_options(${PROJECT_NAME}-cuda PRIVATE ${NVCC_FLAGS})
else()
    #===========================================================
    # If cuda is not enabled, compile .cu files as .cpp
    #===========================================================
    set_source_files_properties(${SRC_CUDA} PROPERTIES LANGUAGE CXX)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      add_definitions("-x c++")
    endif()
    add_library(${PROJECT_NAME}-cuda ${SRC_CUDA} )
    target_compile_options(${PROJECT_NAME}-cuda PRIVATE ${CXX_FLAGS})    
endif()
#===========================================================
# Adding the sim-tools home directory to all files
#===========================================================
target_compile_definitions(${PROJECT_NAME} PUBLIC SIM_TOOLS_HOME_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(${PROJECT_NAME}-cuda PUBLIC SIM_TOOLS_HOME_DIR="${CMAKE_CURRENT_SOURCE_DIR}")


#This triggers compile error if Eigen stuff that is not properly licenced is included. PUBLIC ensures it is visible to libariies linked against sim-tools
target_compile_definitions(${PROJECT_NAME} PUBLIC EIGEN_MPL2_ONLY) 
#target_compile_options(${PROJECT_NAME} PUBLIC --expt-relaxed-constexpr) #removes annoying warnings produced by eigen when compiling cuda


target_include_directories(${PROJECT_NAME}
PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/vendor
    
   # PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/sim-tools
   # PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/vendor/assimp/include
)

target_include_directories(${PROJECT_NAME}-cuda PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

#Link with manually built libs
set(YAML_CPP_BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/yaml-cpp-0.8.0/build-yaml)
target_link_libraries(${PROJECT_NAME} PUBLIC ${YAML_CPP_BUILD_DIR}/libyaml-cpp.a)


target_link_libraries(${PROJECT_NAME} PRIVATE assimp)
target_link_libraries(${PROJECT_NAME} PRIVATE Freetype::Freetype) 
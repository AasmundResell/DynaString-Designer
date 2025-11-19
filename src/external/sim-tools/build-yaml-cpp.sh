#!/bin/bash

#===========================================================
# Builds yaml-cpp if it is not already built
#===========================================================
echo "Building yaml-cpp.."
cd vendor/yaml-cpp*
mkdir -p build-yaml
cd build-yaml
cmake [-DYAML_BUILD_SHARED_LIBS=OFF] ..
make 


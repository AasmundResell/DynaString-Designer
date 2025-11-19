# sim-tools

Utility library that can be useful when building numerical simulation software. It uses yaml-cpp and eigen which are bundled, so the user doesn't need to install these, and can use eigen as a linalg library directly. 

The library implements functionality for parsing yaml files built on top of yaml-cpp.
It also implmenents a custom Quaternion class

## Building the Library

The library is intended to be bundled and not installed. You can include it in your project by either copying the files manually or using Git submodules. 

To integrate `sim-tools` into your project, use CMake as follows:

1. Add the subdirectory in your `CMakeLists.txt`:
    ```cmake
    add_subdirectory(<path-from-your-project's-CMakeLists.txt-to-sim-tools-directory>)
    ```

2. Link the library to your target:
    ```cmake
    target_link_libraries(${PROJECT_NAME} PRIVATE sim-tools)
    ```

## Notes

- Only CMake is supported for building.
- Ensure you use the correct path to the `sim-tools` directory in your `add_subdirectory` command.



Various graphics packages are needed, which include OpenGL, glew, glfw, assimp, freetype
How to isntall required graphics libs when on Ubuntu:

sudo apt install libgl1-mesa-dev &&
sudo apt install libglew-dev libglfw3-dev &&
sudo apt install libassimp-dev libfreetype6-dev


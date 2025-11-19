clear
mkdir -p build-test
mkdir -p build-lib
cd build-test
cmake ..
make 
if [ $? -ne 0 ]; then
    echo "Build failed"
    exit 1
fi
cd ..
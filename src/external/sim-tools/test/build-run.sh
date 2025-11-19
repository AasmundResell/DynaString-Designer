source build.sh
if [ $? -eq 0 ]; then
    build-test/exe testfile.yml
else 
    echo "Build failed"
    exit 1
fi
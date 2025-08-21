rm -rf build/
mkdir build && cd build
cmake .. -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release \
        -DOpenCV_DIR=/usr/local/lib64/cmake/opencv4
cmake --build .
cd ..
make -j$(nproc)

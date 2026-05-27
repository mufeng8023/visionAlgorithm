rm -r build
mkdir build

project_root=$(pwd)

cd build
cmake .. -DPROJECT_ROOT=$project_root
make -j12
make clean
make -j12
cd ..

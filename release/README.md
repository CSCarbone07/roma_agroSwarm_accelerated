# informationGain

mkdir build
cd build
cmake ..
make -j4

./MACPP -i ../test.yaml


if you get a compile error for harfbuzz delete the build folder, repeat and instead of "cmake .." run the following
cmake -DFT_WITH_ZLIB=ON -DCMAKE_DISABLE_FIND_PACKAGE_HarfBuzz=TRUE ..





for IG testing run

./Test_IG






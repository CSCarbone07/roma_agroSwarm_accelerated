# informationGain

Inside release folder:

To build:
mkdir build
cd build
cmake ..
make -j4

if you get a compile error for harfbuzz delete the build folder, repeat and instead of "cmake .." run the following
cmake -DFT_WITH_ZLIB=ON -DCMAKE_DISABLE_FIND_PACKAGE_HarfBuzz=TRUE ..

To do one run:
./MACPP -i ../test.yaml


To run IG testing file:
./Test_IG



For multiple runs enter folder 

In my computer (Carlos Carbone):
cd release/runs/cscarbone
./IGruns_greedy_0_0.sh

To run them on your computer you need to change the paths in the sh and yaml files to match your file location. However, an alternative to avoid this is to use Dockers with the docker file I included and run the image using paths to match any of the run folders paths. For example:

In the root folder where the Docker file is
To build docker image:
docker image build -t cscarbone07:swarm_istc .

To run the container (remember to use your path):
docker container run -it -v $HOME/<yourPathTo"01_UAVswarmInspectionSimulator">:/home/cscarbone cscarbone07:swarm_istc 

Inside the container the paths should match those of my computer and you can safely run the files from release/runs/cscarbone :
cd /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/runs/IG/cscarbone/base_10_50runs
./IGruns_greedy_0_0.sh

You might want to recompile the code inside the docker container if you get issues. For this, delete all files from the release/build folder and run the same build commands but inside the terminal with the docker container, that is:

cd build
cmake ..
make -j4






In any of the cases the results will be saved in the folder
cd release/results


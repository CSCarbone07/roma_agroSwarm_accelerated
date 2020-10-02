
FROM ubuntu:18.04


RUN apt-get update \
&& apt-get install -y \
cmake \
apt-utils \
libxrandr-dev \
libxinerama-dev \
libxcursor-dev \
libyaml-cpp-dev \
libboost-all-dev \
libpng-dev \
libbz2-dev \
libeigen3-dev \
glew-utils \
libglew-dev \
libglu1-mesa-dev \
freeglut3-dev \
mesa-common-dev \
libxi-dev \
libboost-program-options-dev

CMD ["/bin/bash"]

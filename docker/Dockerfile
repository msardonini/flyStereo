# FROM nvidia/cuda:11.4.3-cudnn8-devel-ubuntu20.04
FROM nvidia/cuda:11.6.2-cudnn8-devel-ubuntu20.04


RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y cmake vim xz-utils git libgstreamer1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-plugins-base gstreamer1.0-tools gstreamer1.0-libav libgstreamer-plugins-base1.0-dev python3-dev python3-pip gdb libsqlite3-dev libgtk2.0-dev clang-format libgtest-dev libbenchmark-dev lcov
RUN pip3 install numpy pre-commit
RUN ln -sf /usr/bin/gcov-11 /usr/bin/gcov

# Build and install Yaml-Cpp
RUN cd /opt && \
  git clone https://github.com/jbeder/yaml-cpp.git && \
  cd yaml-cpp && \
  git checkout yaml-cpp-0.6.3 && \
  mkdir build && \
  cd build && \
  cmake -DYAML_BUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF .. && \
  make install

# Build and install Eigen
RUN cd /opt && \
  git clone https://gitlab.com/libeigen/eigen.git && \
  cd eigen && \
  git checkout 3.4-rc1 && \
  mkdir build && \
  cd build && \
  cmake .. && \
  make install

# Build and install VTK
RUN apt-get install -y libxt-dev
RUN cd /opt && \
  git clone https://github.com/Kitware/VTK.git && \
  cd VTK && \
  git checkout v8.2.0 && \
  mkdir build && \
  cd build && \
  cmake .. && \
  make -j$('nproc') && \
  make install

# Build and install OpenCV
RUN cd /opt && \
  git clone https://github.com/opencv/opencv.git && \
  git clone https://github.com/opencv/opencv_contrib.git && \
  cd opencv_contrib && \
  git checkout 4.4.0 && \
  cd ../opencv && \
  git checkout 4.4.0 && \
  mkdir build && \
  cd build && \
  cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D WITH_CUDA=ON \
        -D CUDA_ARCH_BIN=6.1 \
        -D CUDA_ARCH_PTX="" \
        -D ENABLE_FAST_MATH=ON \
        -D CUDA_FAST_MATH=ON \
        -D WITH_CUBLAS=ON \
        -D WITH_LIBV4L=ON \
        -D WITH_V4L=ON \
        -D WITH_VTK=ON \
        -D WITH_GSTREAMER=ON \
        -D WITH_GSTREAMER_0_10=OFF \
        -D WITH_OPENGL=ON \
        -D BUILD_opencv_python2=OFF \
        -D BUILD_opencv_python3=ON \
        -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
        .. && \
  make -j$('nproc') && \
  make install

# Build and install Spdlog
RUN cd /opt && \
  git clone https://github.com/gabime/spdlog.git && \
  cd spdlog && \
  git checkout v1.8.1 && \
  mkdir build && \
  cd build && \
  cmake .. && \
  make -j$('nproc') && \
  make install

# Install the python script that will show eigen matrices when running gdb
COPY docker/printers /opt/printers
RUN mv /opt/printers/.gdbinit /root


# Install VPI
RUN apt-get install -y gnupg && \
  apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc && \
  apt-get install -y software-properties-common && \
  add-apt-repository 'deb https://repo.download.nvidia.com/jetson/x86_64/focal r34.1 main' && \
  apt-get update && \
  apt-get install -y libnvvpi2 vpi2-dev

# Install NSight
RUN apt-get install -y qt5-default cuda-nsight-systems-11-6 nsight-systems-2021.3.2  ca-certificates         dbus         fontconfig         gnupg         libasound2         libfreetype6         libglib2.0-0         libnss3         libsqlite3-0         libx11-xcb1         libxcb-glx0         libxcb-xkb1         libxcomposite1         libxcursor1         libxdamage1         libxi6         libxml2         libxrandr2         libxrender1         libxtst6         openssh-client         wget         xcb         xkb-data


#Install g++11 TODO move to top
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
  apt-get update && \
  apt-get install -y gcc-11 g++-11

RUN apt-get install -y libglew-dev libboost-serialization1.71-dev
# Install Pangolin for ORB_SLAM3
RUN cd /opt && \
  git clone https://github.com/stevenlovegrove/Pangolin.git && \
  cd Pangolin && \
  git checkout d9daba62 && \
  mkdir build && \
  cd build && \
  cmake .. && \
  make -j$('nproc') && \
  make install

# TODO: compile ORB_SLAM3


ENV CXX=/usr/bin/g++-11
ENV CC=/usr/bin/gcc-11

RUN cd /opt && \
  git clone --recursive https://github.com/luxonis/depthai-core.git --branch v2.17.1 && \
  cd depthai-core && \
  cmake -S. -Bbuild -D'CMAKE_INSTALL_PREFIX=/usr/local' && \
  cmake --build build --target install --parallel $('nproc')

FROM nvcr.io/nvidia/l4t-base:r32.7.1

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y cmake vim xz-utils git libgstreamer1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-plugins-base gstreamer1.0-tools gstreamer1.0-libav libgstreamer-plugins-base1.0-dev python3-dev python3-pip gdb libsqlite3-dev clang-format
RUN pip3 install numpy pre-commit


# Build and install Eigen
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
        -D CUDA_ARCH_BIN=5.3 \
        -D CUDA_ARCH_PTX="" \
        -D ENABLE_FAST_MATH=ON \
        -D CUDA_FAST_MATH=ON \
        -D WITH_CUBLAS=ON \
        -D WITH_LIBV4L=ON \
        -D WITH_V4L=ON \
        -D WITH_VTK=OFF \
        -D WITH_GSTREAMER=ON \
        -D WITH_GSTREAMER_0_10=OFF \
        -D WITH_OPENGL=OFF \
        -D BUILD_opencv_python2=OFF \
        -D BUILD_opencv_python3=ON \
        -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
        .. && \
  make -j$('nproc') && \
  make install

# Build and install OpenGV
RUN cd /opt && \
  git clone https://github.com/laurentkneip/opengv.git && \
  cd opengv && \
  mkdir build && \
  cd build && \
  cmake -D CMAKE_BUILD_TYPE=RELEASE .. && \
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
  make install

# Install the python script that will show eigen matrices when running gdb
COPY printers /opt/printers
RUN mv /opt/printers/.gdbinit /root


# RUN pip3 install pre-commit && apt-get install -y clang-format

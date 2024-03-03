FROM ubuntu:focal

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata

# Install dependencies 
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    build-essential \
    git \
    cmake \
    wget \
    libeigen3-dev \
    libboost-all-dev \
    libgmp3-dev \
    libflann-dev \
    ca-certificates \
    freeglut3-dev \
    libssl-dev \
    libgl1-mesa-dev \
    libxt-dev \
    libglu1-mesa-dev \
    mesa-common-dev \
    libusb-1.0-0-dev \
    libusb-dev \
    libopenni-dev \
    libopenni2-dev \
    libpcap-dev \
    libpng-dev \
    mpi-default-dev \
    openmpi-bin \
    openmpi-common \
    libqhull-dev \
    libgtest-dev \
    libopencv-dev \
    libnanoflann-dev

# Install VTK
RUN wget https://www.vtk.org/files/release/8.2/VTK-8.2.0.tar.gz \
    && tar -xf VTK-8.2.0.tar.gz \
    && cd VTK-8.2.0 && mkdir build && cd build \
    && cmake .. -DVTK_MODULE_ENABLE_VTK_RenderingContextOpenGL2=YES \
                -DCMAKE_BUILD_TYPE=Release \
    && make -j5 \
    && make install


# Clone PCL and install it
RUN git clone https://github.com/PointCloudLibrary/pcl.git && \
    cd pcl && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j5 && \
    make install

RUN apt-get update && apt-get install -y pcl-tools

# Set the working directory
WORKDIR /workspace

# Copy the source code
COPY ./src /workspace/src
COPY ./CMakeLists.txt /workspace/CMakeLists.txt
COPY ./cmake /workspace/cmake

# Build the project
RUN mkdir build && \
    cd build && \
    cmake .. && \
    make -j4

# Remove the source code
RUN rm -rf /workspace/src && \
    rm /workspace/CMakeLists.txt && \
    rm -rf /workspace/cmake

# Define the command to run the application
ENTRYPOINT ["/workspace/build/bin/examples"]
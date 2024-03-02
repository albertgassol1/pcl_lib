FROM ubuntu:focal

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# Install dependencies 
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    build-essential \
    git \
    cmake \
    libeigen3-dev \
    libboost-all-dev \
    libgmp3-dev \
    libflann-dev \
    ca-certificates \
    freeglut3-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone PCL and install it
RUN git clone https://github.com/PointCloudLibrary/pcl.git && \
    cd pcl && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j10 && \
    make install

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
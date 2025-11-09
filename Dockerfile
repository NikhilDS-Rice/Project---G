"""
Dockerfile for OMPL-based SMR project.
Includes all dependencies: OMPL, Python packages, etc.
"""

FROM ubuntu:22.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3 \
    python3-pip \
    python3-dev \
    libboost-all-dev \
    libeigen3-dev \
    libode-dev \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install OMPL
WORKDIR /tmp
RUN git clone https://github.com/ompl/ompl.git && \
    cd ompl && \
    mkdir -p build/Release && \
    cd build/Release && \
    cmake ../.. -DPYTHON_EXEC=/usr/bin/python3 && \
    make -j4 && \
    make install && \
    cd /tmp && \
    rm -rf ompl

# Set up Python environment
RUN pip3 install --upgrade pip

# Install Python dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

# Set working directory
WORKDIR /workspace

# Copy project files
COPY . /workspace/

# Set Python path
ENV PYTHONPATH=/usr/local/lib/python3/dist-packages:$PYTHONPATH
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Default command
CMD ["/bin/bash"]

# Use an official image as a base image
FROM ubuntu:latest

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev

# Set the working directory
WORKDIR /app

# Copy the C++ project files into the container
COPY . .

# Build your C++ project
RUN mkdir outputs
RUN rm -rf build && mkdir build && cd build && cmake .. && make


# Specify the default command to run when the container starts
CMD ["./build/bin/test_main"]


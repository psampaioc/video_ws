# ROS 2 Jazzy + CUDA 12.6 + TensorRT 10 + ONNX Runtime 1.19.2
FROM --platform=linux/amd64 nvidia/cuda:12.6.3-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Base development dependencies and ROS 2 repository.
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl wget gnupg2 ca-certificates software-properties-common locales \
    build-essential cmake pkg-config git \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    python3-rosdep python3-colcon-common-extensions python3-pip python3-venv \
    libopencv-dev nlohmann-json3-dev libcudnn9-dev-cuda-12 \
    ros-jazzy-geographic-msgs ros-jazzy-geodesy ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

RUN getent group 992 || groupadd -g 992 host_gpu

# ONNX Runtime C++ GPU libraries.
RUN cd /tmp && \
    wget -q "https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-linux-x64-gpu-1.19.2.tgz" && \
    tar -xzf "onnxruntime-linux-x64-gpu-1.19.2.tgz" && \
    cp -r onnxruntime-linux-x64-gpu-1.19.2/include/* /usr/local/include/ && \
    cp -r onnxruntime-linux-x64-gpu-1.19.2/lib/* /usr/local/lib/ && \
    ldconfig && rm -rf /tmp/onnxruntime*

RUN pip3 install --no-cache-dir \
    onnxruntime-gpu==1.19.2 --extra-index-url https://aiinfra.pkgs.visualstudio.com/PublicPackages/_packaging/onnxruntime-cuda-12/pypi/simple/ \
    tensorrt==10.0.1 onnx onnxsim --break-system-packages

# Local OCR assets are required by V2 but intentionally not published.
RUN mkdir -p /opt/ocr_models
COPY docker/models/v3_en_rec.onnx /opt/ocr_models/
COPY docker/models/en_dict.txt /opt/ocr_models/

ENV CUDA_PATH=/usr/local/cuda
ENV PATH=${CUDA_PATH}/bin:${PATH}
ENV TRT_LIB_PATH=/usr/local/lib/python3.12/dist-packages/tensorrt_libs
ENV LD_LIBRARY_PATH=${TRT_LIB_PATH}:/usr/local/cuda/lib64:/usr/local/lib:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
ENV TRT_CACHE_DIR=/workspace/trt_cache

RUN mkdir -p /workspace/trt_cache && chmod 755 /workspace/trt_cache
RUN rosdep init 2>/dev/null || true && rosdep update
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=${TRT_LIB_PATH}:/usr/local/cuda/lib64:/usr/local/lib:\${LD_LIBRARY_PATH}" >> /root/.bashrc && \
    echo "export CUDA_PATH=/usr/local/cuda" >> /root/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]

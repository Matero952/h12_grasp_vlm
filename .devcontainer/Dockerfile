FROM osrf/ros:humble-desktop-full

WORKDIR /ros2_ws

# Remove any old/expired ROS 2 apt entry
RUN rm /etc/apt/sources.list.d/ros2-latest.list || true


RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      curl gnupg2 lsb-release \
      iputils-ping \
      net-tools \
      build-essential gfortran libatlas-base-dev \
      python3 python3-pip python-is-python3 && \
    rm -rf /var/lib/apt/lists/*

# RUN pip3 install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128

# Re-import ROS 2 key and repo
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
#     | gpg --dearmor \
#     | tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null && \
#     echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
#       http://packages.ros.org/ros2/ubuntu \
#       $(lsb_release -cs) main" \
#     | tee /etc/apt/sources.list.d/ros2-latest.list

# Install Cyclone DDS
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Install RealSense + ROS 2 packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-librealsense2* \
      ros-humble-realsense2-camera-msgs \
      ros-humble-realsense2-camera && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
      "numpy<2" \
      opencv-python \
      open3d \
      matplotlib \
      transformers \
      google-genai \
      Pillow 
      # unitree-sdk2py


# RUN apt-get update && apt-get install -y nano
RUN pip3 install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128


# Install SAM2 repo
#WORKDIR /opt/sam2
#RUN git clone https://github.com/facebookresearch/sam2.git . && \
#    pip3 install --no-cache-dir -e .
WORKDIR /ros2_ws

# Install Ultralytics + CLIP
#RUN pip install --no-cache-dir ultralytics \
# && pip install --no-cache-dir git+https://github.com/openai/CLIP.git

# Set CycloneDDS as RMW implementation
ENV CYCLONEDDS_URI=file:///ros2_ws/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# ENV CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"wlp2s0\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
COPY h12_grasp_vlm1.png /ros2_ws/h12_grasp_vlm1.png

# Entry point script to source and conditionally build
RUN echo '#!/usr/bin/env bash' > /ros_entrypoint.sh && \
    echo 'set -e' >> /ros_entrypoint.sh && \
    echo 'echo "Sourcing ROS 2..."' >> /ros_entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /ros_entrypoint.sh && \
    echo 'if [ -d /ros2_ws/src/vision_pipeline ]; then' >> /ros_entrypoint.sh && \
    echo '  echo "Building vision_pipeline...";' >> /ros_entrypoint.sh && \
    echo '  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release;' >> /ros_entrypoint.sh && \
    echo 'fi' >> /ros_entrypoint.sh && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then' >> /ros_entrypoint.sh && \
    echo '  echo "Sourcing overlay workspace...";' >> /ros_entrypoint.sh && \
    echo '  source /ros2_ws/install/setup.bash;' >> /ros_entrypoint.sh && \
    echo 'fi' >> /ros_entrypoint.sh && \
    echo 'exec "$@"' >> /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# RUN echo 'export CYCLONEDDS_URI=file:///ros2_ws/cyclonedds.xml >> ~/.bashrc'

# Create setup.bash with the desired content

# Set Cyclone DDS config
# RUN export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>"

# Check for missing dependencies (optional)
# rosdep install -i --from-path src --rosdistro humble -y
# Build workspace
# RUN echo "Cache bust: $(date)" > /tmp/cachebust
RUN echo 'PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages' > /ros2_ws/.env


RUN apt update
RUN apt install --only-upgrade ros-humble-geometry-msgs
RUN apt update
RUN apt install nano
WORKDIR /opt/sam2
RUN git clone https://github.com/facebookresearch/sam2.git . && \
    pip3 install --no-cache-dir -e .
WORKDIR /ros2_ws

WORKDIR /ros2_ws

# Install Ultralytics + CLIP
RUN pip install --no-cache-dir ultralytics \
 && pip install --no-cache-dir git+https://github.com/openai/CLIP.git

RUN apt update
RUN pip install pin
RUN pip install meshcat
RUN pip install meshcat-shapes
RUN pip install pin-pink
RUN pip install unitree_sdk2py

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
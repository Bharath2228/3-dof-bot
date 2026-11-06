FROM osrf/ros:jazzy-desktop-full

WORKDIR /ros_ws
COPY src/ src/

RUN apt-get update && apt-get install -y python3-pip libssl-dev git && rosdep update && rosdep install --from-paths src --ignore-src -r -y

RUN pip3 install --no-cache-dir --break-system-packages python-dotenv flask flask-ask-sdk ask-sdk-core ask-sdk-model \
    && pip3 uninstall -y --break-system-packages oscrypto \
    && pip3 install --no-cache-dir --break-system-packages git+https://github.com/wbond/oscrypto.git@1547f535001ba568b239b8797465536759c742a3

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
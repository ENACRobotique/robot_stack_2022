FROM ros:galactic

# install ros package
RUN apt-get update && apt-get install -y \
      ros-galactic-rmw-cyclonedds-cpp \
      ros-galactic-demo-nodes-cpp \
      ros-galactic-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
ENV HOST_ADDR="1.2.3.4"

# Using participant index
ENV CYCLONEDDS_URI="<CycloneDDS><Domain id='any'><General><ExternalNetworkAddress>${HOST_ADDR}</ExternalNetworkAddress><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>1</ParticipantIndex><Peers><Peer address='${HOST_ADDR}'/></Peers></Discovery><Tracing><Verbosity>config</Verbosity><Out>stderr</Out></Tracing></Domain></CycloneDDS>"

# launch ros package
CMD ["ros2", "run", "demo_nodes_py", "talker"]

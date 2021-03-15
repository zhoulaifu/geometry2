FROM ros:foxy
ARG GIT_ACCESS_TOKEN



RUN apt-get update \
         && apt-get install -y  lcov afl++ flex

ENV MOUNT /mnt/local
ENV TEEX /opt/teex

RUN mkdir -p $MOUNT
RUN mkdir -p $TEEX

WORKDIR $MOUNT


RUN git config --global url."https://${GIT_ACCESS_TOKEN}:@github.com/".insteadOf "https://github.com/"
RUN git clone https://github.com/zhoulaifu/21_teex $TEEX
RUN cd $TEEX/shaping && make main_tool



RUN  . /opt/ros/${ROS_DISTRO}/setup.sh \
        && VERBOSE=1 CXX="afl-g++" CXXFLAGS="--coverage -g -fsanitize=address,undefined -fsanitize-undefined-trap-on-error" colcon build --event-handlers console_direct+

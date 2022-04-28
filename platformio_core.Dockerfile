FROM ubuntu:latest

SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update && apt-get install -y \
    python3-pip \
    libffi-dev \
    gcc \
    musl-dev

ENV CRYPTOGRAPHY_DONT_BUILD_RUST=1

RUN pip install platformio

# folowing command installs more dependencies...
RUN pio remote agent start || true

#TODO : move it to the first RUN
#RUN apk add --no-cache eudev-dev

ENTRYPOINT [ "pio", "remote", "agent", "start" ]
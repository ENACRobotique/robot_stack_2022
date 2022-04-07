#This version doesn't work on linux -> Use 'ubuntu' instead

FROM python:3.8-alpine

RUN apk add --no-cache libffi-dev openssl-dev gcc musl-dev libc6-compat gcompat

ENV CRYPTOGRAPHY_DONT_BUILD_RUST=1

RUN pip3 install platformio

# folowing command installs more dependencies...
RUN pio remote agent start || true

#TODO : move it to the first RUN
RUN apk add --no-cache eudev-dev

ENTRYPOINT [ "pio", "remote", "agent", "start" ]
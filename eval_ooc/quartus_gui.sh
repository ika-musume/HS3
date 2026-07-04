#!/bin/bash
#Launch Quartus 17.0 GUI via Docker

set -e
xhost +local:docker > /dev/null

PROJECT_DIR="${1:-$(pwd)}"
docker run --rm \
  -v "$PROJECT_DIR":/host \
  -w /host \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY="$XAUTHORITY" \
  -v "$XAUTHORITY":"$XAUTHORITY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e GALLIUM_DRIVER=softpipe \
  --net=host \
  raetro/quartus:17.0 \
  quartus &

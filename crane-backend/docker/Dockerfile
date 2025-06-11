FROM ubuntu:22.04 AS build

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libboost-system-dev \
    libboost-program-options-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

COPY . /backend
WORKDIR /backend/build
RUN cmake .. && cmake --build . && CTEST_OUTPUT_ON_FAILURE=TRUE cmake --build . --target test

FROM ubuntu:22.04

RUN groupadd -r server && useradd -r -g server server
USER server

COPY --from=build /backend/build/bin /application
COPY --from=build /backend/build/config /application
WORKDIR /application

EXPOSE 8081
CMD ["./app", "--config-file=robot_config.json", "--port=8081"]
FROM seajackal/hydrolib:latest

ARG USERNAME=hydrolib
ARG UID=1000
ARG GID=$UID

RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add - \
    && echo "deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-19 main" \
    | sudo tee /etc/apt/sources.list.d/llvm.list \
    && apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    openocd \
    ca-certificates\
    sudo \
    gdb-multiarch \
    python3-pip \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN pip install compdb==0.2.0

ENV GCC_ARM_VERSION=14.2.1-1.1
RUN wget -q https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v${GCC_ARM_VERSION}/xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}-linux-x64.tar.gz \
    && tar -xf xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}-linux-x64.tar.gz -C /opt \
    && rm xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}-linux-x64.tar.gz \
    && cp -s /opt/xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}/bin/* /usr/local/bin/ \
    && ln -s /opt/xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}/arm-none-eabi/include /usr/include/newlib

ARG RENODE_VERSION=1.16.0
    
RUN wget https://github.com/renode/renode/releases/download/v${RENODE_VERSION}/renode_${RENODE_VERSION}_amd64.deb && \
    apt-get update && \
    apt-get install -y --no-install-recommends ./renode_${RENODE_VERSION}_amd64.deb python3-dev && \
    rm ./renode_${RENODE_VERSION}_amd64.deb && \
    rm -rf /var/lib/apt/lists/*
RUN pip3 install -r /opt/renode/tests/requirements.txt --no-cache-dir


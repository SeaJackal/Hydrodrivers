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
    automake \
    autoconf \
    libtool \ 
    policykit-1 \
    libgtk-3-dev \
    uml-utilities \
    gtk-sharp3 \
    dotnet-sdk-8.0 \
    dotnet-runtime-8.0 \
    zlib1g \    
    snapd \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

    
RUN pip install compdb==0.2.0

ENV GCC_ARM_VERSION=14.2.1-1.1
RUN wget -q https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v${GCC_ARM_VERSION}/xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}-linux-x64.tar.gz \
    && tar -xf xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}-linux-x64.tar.gz -C /opt \
    && rm xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}-linux-x64.tar.gz \
    && cp -s /opt/xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}/bin/* /usr/local/bin/ \
    && ln -s /opt/xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}/arm-none-eabi/include /usr/include/newlib

    
RUN git clone https://github.com/renode/renode.git && \
    cd renode && \
    python3 -m pip install -r tests/requirements.txt && \
    chmod +x build.sh && \
    ./build.sh
RUN ln -s /renode/renode /usr/local/bin/renode
ENV export PATH="$PATH:/renode"


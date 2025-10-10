FROM mcr.microsoft.com/devcontainers/base:jammy

ARG USERNAME=hydrolib
ARG UID=1001
ARG GID=$UID

# Install some dependencies packages
RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add - \
    && echo "deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-19 main" \
    | sudo tee /etc/apt/sources.list.d/llvm.list \
    && apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    build-essential \
    gdb \
    cmake \
    libgtest-dev \
    doxygen \
    clang-format-19 \
    clangd-19 \
    openocd \
    gdb-multiarch \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
    && update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-19 100 \
    && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-19 100

# Install latest gcc-arm-none-eabi from ARM Developer
ENV GCC_ARM_VERSION=14.2.1-1.1
RUN wget -q https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v${GCC_ARM_VERSION}/xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}-linux-x64.tar.gz \
    && tar -xf xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}-linux-x64.tar.gz -C /opt \
    && rm xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}-linux-x64.tar.gz \
    && cp -s /opt/xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}/bin/* /usr/local/bin/ \
    && ln -s /opt/xpack-arm-none-eabi-gcc-${GCC_ARM_VERSION}/arm-none-eabi/include /usr/include/newlib

FROM mcr.microsoft.com/devcontainers/base:1-ubuntu-22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ORIGINAL_USER=vscode
ARG NEW_USER=madelink

# Install additional packages
RUN apt update && apt install -y \
    wget \
    figlet \
    cmake \
    lcov \
    gdb \
    sudo

# Install figlet font
RUN cd /usr/share/figlet && wget https://raw.githubusercontent.com/xero/figlet-fonts/master/3d.flf

# Install Unity testing framework v2.6.1
RUN cd /opt && \
    git clone --depth 1 --branch v2.6.1 https://github.com/ThrowTheSwitch/Unity.git unity && \
    chown -R root:root unity/

# Change user name from vscode to madelink
RUN usermod -l ${NEW_USER} ${ORIGINAL_USER} && \
    groupmod -n ${NEW_USER} ${ORIGINAL_USER} && \
    usermod -d /home/${NEW_USER} -m ${NEW_USER}

# Allow all users in the sudo group to use sudo without a password
RUN adduser ${NEW_USER} sudo
RUN echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/nopasswd && \
    chmod 0440 /etc/sudoers.d/nopasswd

WORKDIR /tmp

# Install clang-format-18
WORKDIR /tmp
RUN apt update && apt install -y lsb-release software-properties-common gnupg 
RUN wget https://apt.llvm.org/llvm.sh
RUN chmod +x llvm.sh
RUN ./llvm.sh 18 clang-format
RUN apt install -y clang-format-18
RUN ln -s /usr/bin/clang-format-18 /usr/bin/clang-format
RUN rm llvm.sh

RUN rm -rf /tmp/*

# Setup user and home
ENV USER=${NEW_USER}
ENV HOME=/home/${NEW_USER}
WORKDIR ${HOME}
USER ${NEW_USER}

# Install the bashrc addons
COPY .bashrc_devcontainer_addons ${HOME}/.bashrc_devcontainer_addons

# Don't display message about how to use sudo...
RUN touch ${HOME}/.sudo_as_admin_successful

CMD ["/bin/bash", "-c"]
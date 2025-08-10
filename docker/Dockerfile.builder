# ---------- base image with glibc 2.36 ---------------------------------
FROM rockylinux:9

# ---------- developer tool-chain ---------------------------------------
RUN dnf -y groupinstall "Development Tools"  \
    && dnf -y install epel-release           \
    && dnf -y install cmake3 git \
       python3 python3-pip                   \
       opencv opencv-devel                   \
    && ln -sf /usr/bin/cmake3 /usr/local/bin/cmake \
    && dnf clean all

# ---------- optional: a non-root user ----------------------------------
ARG USER=builder UID=1000
RUN useradd -m -u ${UID} ${USER}
USER ${USER}
WORKDIR /home/${USER}/work            # mount your source tree here

# ---------- default shell ---------- (so `docker run` drops you in)
CMD ["/bin/bash"]

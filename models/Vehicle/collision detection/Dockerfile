# hash:sha256:8794aa0876e7c966ced2ed8df66eaf96a49cd3fcf59055e71075b1f221082f79
FROM registry.codeocean.com/codeocean/matlab:2019a-ubuntu18.04

ARG DEBIAN_FRONTEND=noninteractive
ARG MLM_LICENSE_FILE

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential=12.4ubuntu1 \
        cmake=3.10.2-1ubuntu2.18.04.2 \
        curl=7.58.0-2ubuntu3.24 \
        ffmpeg=7:3.4.11-0ubuntu0.1 \
        git=1:2.17.1-1ubuntu0.17 \
        imagemagick=8:6.9.7.4+dfsg-16ubuntu6.15 \
        libffi-dev=3.2.1-8 \
        libgdbm-dev=1.14.1-6 \
        libncurses5-dev=6.1-1ubuntu1.18.04 \
        libnss3-dev=2:3.35-2ubuntu2.16 \
        libreadline-dev=7.0-3 \
        libssl-dev=1.1.1-1ubuntu2.1~18.04.21 \
        python3-pip=9.0.1-2.3~ubuntu1.18.04.8 \
        python3-setuptools=39.0.1-2ubuntu0.1 \
        python3-wheel=0.30.0-0.2ubuntu0.1 \
        python3.7-dev=3.7.5-2ubuntu1~18.04.2 \
        python3.7-distutils \
        tar=1.29b-2ubuntu0.4 \
        unzip=6.0-21ubuntu1.2 \
        wget=1.19.4-1ubuntu2.2 \
        zlib1g-dev=1:1.2.11.dfsg-0ubuntu2.2 \
    && rm -rf /var/lib/apt/lists/*

COPY postInstall /
RUN /postInstall

COPY run.sh /
COPY check_collision.py /
COPY BEL_Putte-4_2_T-1.xml /code/
COPY BEL_Putte-4_2_T-1_occupancies.csv /results/
RUN /run.sh

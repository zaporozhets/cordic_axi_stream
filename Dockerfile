FROM ubuntu:24.04

# Setup environment
ENV DEBIAN_FRONTEND noninteractive


RUN apt-get update
RUN apt-get install -y --no-install-recommends \
        wget \
        sudo \
        git \
        python3 \
        python3-pip \
        libpython3.12 \
        iverilog


COPY requirements.txt requirements.txt

RUN pip3 install pylint --break-system-packages
RUN pip3 install -r ./requirements.txt --break-system-packages


ENV VERIBLE_TARBALL="https://github.com/chipsalliance/verible/releases/download/v0.0-3648-g5ef1624a/verible-v0.0-3648-g5ef1624a-linux-static-x86_64.tar.gz"

RUN mkdir -p verible && \
        wget -qO- $VERIBLE_TARBALL | tar -zxvf - -C verible --strip-components=1 && \
        for i in ./verible/bin/*; do sudo cp $i /usr/local/bin/$(basename $i); done
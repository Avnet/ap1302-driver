#!/bin/bash -ex

export ARCH=arm64
export CROSS_COMPILE=/opt/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-

export KERNEL_SRC=../../linux-imx
export CONFIG_VIDEO_AP1302=m

export src=${PWD}

pushd kmod
make $@
popd

pushd fw
for file in $(ls *.xml); do
	./xml2bin.py ${file} ${file%.*}.bin
done
popd

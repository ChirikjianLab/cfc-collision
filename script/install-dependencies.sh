#!/bin/bash

set -e

if [ `id -u` == 0 ]; then
    SUDO=
    export DEBIAN_FRONTEND=noninteractive
    apt-get -y install lsb-release
else
    SUDO="sudo -H"
fi

ubuntu_version=`lsb_release -rs | sed 's/\.//'`

# install common dependencies via apt
${SUDO} apt -y update
${SUDO} apt -y upgrade
${SUDO} apt -y install g++ cmake libboost1.71-all-dev libeigen3-dev git tar
export CXX=g++
export MAKEFLAGS="-j `nproc`"

build_and_install() {
    mkdir build && cd build || exit
    cmake ../"$1"
    make && ${SUDO} make install
    cd .. && rm -rf build
}

mkdir "$HOME"/temp && cd "$HOME"/temp || exit

# install ceres solver from source
${SUDO} apt -y install libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev

wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.0.0.tar.gz
tar -xf 2.0.0.tar.gz && rm 2.0.0.tar.gz
build_and_install "ceres-solver-2.0.0"

# install ifopt solver from source
${SUDO} apt -y install coinor-libipopt-dev

git clone https://github.com/ethz-adrl/ifopt.git
build_and_install "ifopt"

# install fcl 0.6 from source
${SUDO} apt -y install libccd-dev

git clone https://github.com/flexible-collision-library/fcl.git
build_and_install "fcl"

# gtest build and install
${SUDO} apt -y install libgtest-dev

# remove /temp folder
${SUDO} rm -r "$HOME"/temp

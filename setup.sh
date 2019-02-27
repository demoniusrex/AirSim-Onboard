export CXX="g++-6"
export CC="gcc-6"
CMAKE="$(readlink -f AirSim/cmake_build/bin/cmake)"
"$CMAKE" --version


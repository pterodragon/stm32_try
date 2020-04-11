# This builds for all micro-architectures
# For Cortex M4 only 'arm-none-eabi/lib/thumb/v7e-m/fpv4-sp/' matters
TARGET=arm-none-eabi
src_dir=$(realpath ./newlib-cygwin/)
NEWLIB_VERSION=newlib-3.1.0
INSTALL_PREFIX=$src_dir/install/newlib/${TARGET}

flags="--host=`cc -dumpmachine` --build=`cc -dumpmachine` --target=${TARGET} --prefix=$INSTALL_PREFIX --disable-newlib-supplied-syscalls --enable-newlib-reent-small --disable-newlib-fvwrite-in-streamio --disable-newlib-fseek-optimization --disable-newlib-wide-orient --enable-newlib-nano-malloc --disable-newlib-unbuf-stream-opt --enable-lite-exit --enable-newlib-global-atexit --enable-newlib-nano-formatted-io --disable-newlib-fvwrite-in-streamio --disable-nls"

cd newlib-cygwin/ && git checkout $NEWLIB_VERSION && cd ../
if [ -d "build" ]
then
rm -r build
fi

mkdir -p build/newlib install &&
cd build/newlib &&
$src_dir/configure $flags --prefix=$src_dir/install/newlib -v &&
make all -j8

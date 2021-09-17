FROM --platform=amd64 ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
	git \
	bc \
	crossbuild-essential-armel

ENV CROSS_COMPILE=/usr/bin/arm-linux-gnueabi-
ENV ARCH=arm

RUN echo '\
set -e\n\
echo "Building in $(pwd)"\n\
make -j$(nproc)\n\
make -j$(nproc) modules\n\
mkdir -p target/boot\n\
INSTALL_MOD_PATH="target/" make modules_install \n\
cp arch/arm/boot/zImage  target/boot/zImage\n\
cp arch/arm/boot/dts/armada-385-ts*.dtb target/boot/\n\
cd target\n\
tar --owner=0 --group=0 -cJf ../linux-4.4.186-armada-17.02.2-$(git rev-parse --short HEAD).tar.xz .\n\
cd ../\n\
echo "Kernel image written to: linux-4.4.186-armada-17.02.2-$(git rev-parse --short HEAD).tar.xz"\n\
echo "To install kernel, run the following on the SBC: tar -xf linux-4.4.186-armada-17.02.2-$(git rev-parse --short HEAD).tar.xz -C /"\n\
' > /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /usr/src
ENTRYPOINT [ "/bin/bash" ]
CMD [ "/entrypoint.sh" ]

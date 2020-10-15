#!/bin/sh

# Target info
BOARD_TYPE=esp32
BOARD_NAME='AI Thinker ESP32-CAM'
SRC_DIR=`pwd`/src
MAIN_SRC=esp32_main.ino
SRC_LIST="${MAIN_SRC} mqtt.cpp mqtt.h camera_setup.h camera_setup.cpp \
                      local_time.h local_time.cpp debugging.h debugging.cpp"
DEV_IFACE=ttyUSB
LIB_PATH=/home/js/project/arduino/fire_detector
CONFIG_FILE='arduino-cli.yaml'
CONFIG_PATH=`find ~/ -name ${CONFIG_FILE}`
TARGET_TYPE=`arduino-cli board listall ${BOARD_TYPE} | grep "${BOARD_NAME}" | awk {'print $NF'}`
BUILD_DIR=`echo ${MAIN_SRC} | cut -d '.' -f 1`
SRC_UPDATE=0
LIB_PATH_UPDATE=0
DEBUG=0


if [ ! ${CONFIG_PATH} ]; then
	arduino-cli config init
	echo config file create
fi

if [ ${LIB_PATH_UPDATE} -ne 0 ]; then
	cp ${CONFIG_PATH} ${CONFIG_PATH}.bk # backup origin
	sed -i "/user:/c\  user: ${LIB_PATH}" ${CONFIG_PATH}
fi

# --------------------- Compile & Build -----------------------------------------

CONNECTED=`arduino-cli board list | grep /dev/${DEV_IFACE} | awk {'print $1'}`
if [ ! ${CONNECTED} ]; then
	echo "The device isn't connected yet"
	exit 1
fi

if [ ${DEBUG} -ne 0 ]; then
	echo "-----------------------------------------------------"
	echo "Source directory: ${SRC_DIR}"
	echo "Source list: ${SRC_LIST}"
	echo "Target core: ${TARGET_TYPE}"
	echo "Config file path: ${CONFIG_PATH}"
	echo "Build directory: ${BUILD_DIR}"
	echo "Connected device: ${CONNECTED}"
	echo "-----------------------------------------------------"
fi

if [ ! -d ./${BUILD_DIR} ]; then
	mkdir ${BUILD_DIR}
fi

exist_cert=`echo ${SRC_LIST} | grep -w mqtt_cert.h`
if [ exist_cert ]; then
	#cp ${SRC_DIR}/mqtt_cert.h ${BUILD_DIR}/mqtt_cert.h
	cp ${SRC_DIR}/mqtt_cert.h.aws ${BUILD_DIR}/mqtt_cert.h
	#cp ${SRC_DIR}/mqtt_cert.h.bk ${BUILD_DIR}/mqtt_cert.h
fi

src_exist=`ls ${BUILD_DIR} | wc -l`
if [ ${src_exist} -eq 0 -o ${SRC_UPDATE} -ne 0 ]; then
	for src in ${SRC_LIST}; do
		cp ${SRC_DIR}/${src} ${BUILD_DIR}
	done
fi

# - Compile
arduino-cli compile --fqbn ${TARGET_TYPE} ${BUILD_DIR}
if [ $? -ne 0 ]; then
	echo "Compile fail"
	exit 1
fi
echo "Compile ok"

# - Upload firmware in target board
arduino-cli upload -p ${CONNECTED} --fqbn ${TARGET_TYPE} ${BUILD_DIR}
if [ $? -ne 0 ]; then
	echo "Upload fail"
	exit 1
fi
echo "Upload ok"

exit 0


#-------------------- Install "arduino-cli" binary & Set enviroment -------------------------
# - If you use arduino-cli first time, Install binary below command and add $BIN_DIR in 'PATH' environment variable
# BIN_DIR="your bin directory"
# curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$BIN_DIR sh
# arduino-cli core update-index

# -------------------- Check board info & Install board manager -------------------------
# - Check current connected board info
# arduino-cli board list

# - Search install available lists for specific board
# arduino-cli board listall "board type"

# - Install the core for your board
# arduino-cli core install "core name"

# - Display installed core list
# arduino-cli core list

# - Add urls
# arduino-cli core update-index --additional-urls "input url"

# - Search board
# arduino-cli core search "board type" --additional-urls "input url"

# --------------------- Search library & Install --------------------------------
# - Search library
# arduino-cli lib search "lib name"

# - Install library
# arduino-cli lib install "lib name"

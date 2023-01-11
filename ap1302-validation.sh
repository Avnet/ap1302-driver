#!/bin/bash -xe

# Runs different setting on the AP1302

BD_MODEL_MPLUS=$(cat /sys/firmware/devicetree/base/model | grep -a MPLUS > /dev/null && echo 1 || echo 0)
BD_MODEL_MEK=$(cat /sys/firmware/devicetree/base/model | grep -a MEK  > /dev/null && echo 1 || echo 0)

if [ ${BD_MODEL_MPLUS} -eq 1 ]; then

	echo MPLUS Config
	MIPI_CSI="mxc-mipi-csi2.1"
	# # MPLUS 5.10
	VIDEO_MAIN=/dev/v4l/by-path/platform-32c00000.bus:camera-video-index0
	VIDEO_SEC=/dev/v4l/by-path/platform-32c00000.bus:camera-video-index1

elif [ ${BD_MODEL_MEK} -eq 1 ]; then
	
	echo MEK Config
	# MEK
	MIPI_CSI="mxc-mipi-csi2.0"
	#MEK
	VIDEO_MAIN=/dev/v4l/by-path/platform-bus\@58000000\:camera-video-index0
	VIDEO_SEC=/dev/v4l/by-path/platform-bus\@58000000\:camera-video-index1
fi


MODEL=$(cat $(find /sys/firmware/devicetree -name "sensor,model"))

BUS_DEV=$(ls /sys/bus/i2c/drivers/ap1302 | grep 003c)

AP1302_I2C="${BUS_DEV}"
AP1302_DEV="ap1302.${AP1302_I2C}"
AP1302_SENSOR="${AP1302_I2C}.${MODEL}"


function enable_primary() {
	LINK_STATE=$1
	# AP1302 PAD 0 is the primary sensor
	media-ctl -l '"'${AP1302_SENSOR}'.0":0 -> "'${AP1302_DEV}'":0['${LINK_STATE}']'
}

function enable_secondary() {
	LINK_STATE=$1
	# AP1302 PAD 1 is the secondary sensor
	media-ctl -l '"'${AP1302_SENSOR}.1'":0 -> "'${AP1302_DEV}'":1['${LINK_STATE}']'
}
function enable_vc() {
	LINK_STATE=$1
	
	if [ ${BD_MODEL_MPLUS} -eq 1 ]; then
		echo "MPlus does not support virtual channel"
		return 0
	fi
	
	echo "Virtual Channel: $1"
	# AP1302 PAD 3 is the stream of secondary sensor in virtual channel
	media-ctl -l '"'${AP1302_DEV}'":3 -> "'${MIPI_CSI}'":1['${LINK_STATE}']'
}

# Fix issue with wayland
export XDG_RUNTIME_DIR=/run/user/`id -u`

function stream_main() {
	width=$1
	height=$2
	echo "Streaming Primary ${width} x ${height}"
	timeout 20 gst-launch-1.0 v4l2src num-buffers=300 device=${VIDEO_MAIN} ! \
	"video/x-raw,width=${width},height=${height},format=YUY2,framerate=30/1" ! \
	imxvideoconvert_g2d ! \
	waylandsink -v
	#v4l2h264enc ! rtph264pay config-interval=1 pt=96 ! udpsink host=10.102.1.148 port=5000
	#vpuenc_h264 ! rtph264pay config-interval=1 pt=96 ! udpsink host=10.102.1.148 port=5000
	#waylandsink -v
}

function stream_sec() {
	width=$1
	height=$2
	echo "Streaming Secondary ${width} x ${height}"
	timeout 20 gst-launch-1.0 v4l2src num-buffers=300 device=${VIDEO_SEC} ! \
	"video/x-raw,width=${width},height=${height},format=YUY2,framerate=30/1" ! \
	imxvideoconvert_g2d ! \
	waylandsink -v
	#vpuenc_h264 ! rtph264pay config-interval=1 pt=96 ! udpsink host=10.102.1.148 port=5000
	#waylandsink -v
}

function stream_dual() {
	width=$1
	height=$2
	echo "Streaming Dual ${width} x ${height}"
	timeout 20 gst-launch-1.0 -v imxcompositor_g2d name=comp sink_1::xpos=0 sink_1::ypos=${height}  ! waylandsink \
	v4l2src device=${VIDEO_MAIN} num-buffers=300 ! video/x-raw,format=YUY2,width=${width},height=${height},framerate=30/1 ! imxvideoconvert_g2d ! comp. \
	v4l2src device=${VIDEO_SEC}  num-buffers=300 ! video/x-raw,format=YUY2,width=${width},height=${height},framerate=30/1 ! imxvideoconvert_g2d ! comp. -v
}

function ap1302_status() {
	v4l2-ctl -d $(media-ctl -e ${AP1302_DEV}) --log-status
}

case ${MODEL} in
	ar1335)
		# MEK, enough ISI for 4K 2 stream
		WIDTH=2160
		HEIGHT=1440
		
	if [ ${BD_MODEL_MPLUS} -eq 1 ]; then
			# MPLUS ISI only support 2K
			WIDTH=1920
			HEIGHT=1080
	fi
		;;
	ar0144)
		WIDTH=1280
		HEIGHT=800
		;;
	ar0830)
		WIDTH=2160
		HEIGHT=1440
		;;
	*)
		echo "Unknown Model: ${MODEL}"
		WIDTH=1280
		HEIGHT=800
		;;
esac

# If sourced, exit
(return 0 2>/dev/null) && return 0 || echo "Running Streams"

ap1302_status

enable_vc 0
enable_primary 0
enable_secondary 0

ap1302_status


# TPG
stream_main ${WIDTH} ${HEIGHT}

ap1302_status

enable_primary 1
stream_main ${WIDTH} ${HEIGHT}

ap1302_status

enable_secondary 1
stream_main ${WIDTH} $((${HEIGHT}/2))

enable_primary 0
stream_main ${WIDTH} ${HEIGHT}

ap1302_status

return 0

enable_vc 1
enable_primary 0
enable_secondary 0
# TPG
stream_main ${WIDTH} ${HEIGHT}
enable_primary 1
stream_main ${WIDTH} ${HEIGHT}
enable_secondary 1
stream_main ${WIDTH} ${HEIGHT}
stream_sec ${WIDTH} ${HEIGHT}
stream_dual ${WIDTH} ${HEIGHT}

echo "AP1302 Stream Complete"
	
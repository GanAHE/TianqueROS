#!/bin/bash
# run multiple instances of the 'px4' binary, with the gazebo SITL simulation
# It assumes px4 is already built, with 'make px4_sitl_default'

# The simulator is expected to send to TCP port 4560+i for i in [0, N-1]
# For example gazebo can be run like this:
#./Tools/gazebo_sitl_multiple_run.sh -n 10 -m iris

function cleanup() {
	pkill -x px4
	pkill gzclient
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
	echo "Usage: $0 [-n <num_vehicles>] [-m <vehicle_model>]"
	exit 1
fi

while getopts n:m: option
do
	case "${option}"
	in
		n) NUM_VEHICLES=${OPTARG};;
		m) VEHICLE_MODEL=${OPTARG};;
	esac
done

num_vehicles=${NUM_VEHICLES:=3}
export PX4_SIM_MODEL=${VEHICLE_MODEL:=iris}

if [ "$PX4_SIM_MODEL" != "iris" ] && [ "$PX4_SIM_MODEL" != "plane" ] && [ "$PX4_SIM_MODEL" != "standard_vtol" ]
then
	echo "Currently only the following vehicle models are supported! [iris, plane, standard_vtol]"
	exit 1
fi

if [ $num_vehicles -gt 255 ]
then
	echo "Tried spawning $num_vehicles vehicles. The maximum number of supported vehicles is 255"
	exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/.."

build_path=${src_path}/build/px4_sitl_default
mavlink_udp_port=14560
mavlink_tcp_port=4560
world="empty"

echo "killing running instances"
pkill -x px4 || true

sleep 1

source ${src_path}/Tools/setup_gazebo.bash ${src_path} ${src_path}/build/px4_sitl_default

echo "Starting gazebo"
gzserver ${src_path}/Tools/sitl_gazebo/worlds/${world}.world --verbose &
sleep 5

n=0
while [ $n -lt $num_vehicles ]; do
	working_dir="$build_path/instance_$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null
	echo "starting instance $n in $(pwd)"
	../bin/px4 -i $n -d "$src_path/ROMFS/px4fmu_common" -w sitl_${PX4_SIM_MODEL}_${n} -s etc/init.d-posix/rcS >out.log 2>err.log &
	python3 ${src_path}/Tools/sitl_gazebo/scripts/xacro.py ${src_path}/Tools/sitl_gazebo/models/rotors_description/urdf/${PX4_SIM_MODEL}_base.xacro \
		rotors_description_dir:=${src_path}/Tools/sitl_gazebo/models/rotors_description mavlink_udp_port:=$(($mavlink_udp_port+$n)) \
		mavlink_tcp_port:=$(($mavlink_tcp_port+$n))  -o /tmp/${PX4_SIM_MODEL}_${n}.urdf

	gz sdf -p  /tmp/${PX4_SIM_MODEL}_${n}.urdf > /tmp/${PX4_SIM_MODEL}_${n}.sdf
	echo "Spawning ${PX4_SIM_MODEL}_${n}"

	gz model --spawn-file=/tmp/${PX4_SIM_MODEL}_${n}.sdf --model-name=${PX4_SIM_MODEL}_${n} -x 0.0 -y $((3*${n})) -z 0.0

	popd &>/dev/null

	n=$(($n + 1))
done

trap "cleanup" SIGINT SIGTERM EXIT

echo "Starting gazebo client"
gzclient

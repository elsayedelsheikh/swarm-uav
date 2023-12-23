#!/bin/bash
set -e

num_uavs=4 ## Number of UAVs to be simulated
ws_dir=$(pwd)

## Add new parameters for ardupilot instead of gazebo-iris
# Verbose
echo "Adding new parameters for ardupilot instead of gazebo-iris"
cd $ws_dir/ThirdParty/ardupilot/Tools/autotest/pysim

# Create a temporary file for insertion
tmpfile=$(mktemp /tmp/vehicleinfo_tmp.XXXXXX)
trap "rm -f $tmpfile" EXIT

for ((i=0; i<$num_uavs; i++)); do
    echo '            "gazebo-uav'"$i"'": {' >> "$tmpfile"
    echo '                "waf_target": "bin/arducopter",' >> "$tmpfile"
    echo '                "default_params_filename": ["default_params/copter.parm","default_params/gazebo-uav'"$i"'.parm"],' >> "$tmpfile"
    echo '                "external": True,' >> "$tmpfile"
    echo '            },' >> "$tmpfile"
done
## Sed command to add the temporary file content to vehicleinfo.py
sed -i '140r '"$tmpfile" vehicleinfo.py

# Add parameter files by calling the function
# Verbose
echo "Adding parameter files for $num_uavs UAVs"

# Create parameter files in default_params directory
cd $ws_dir/ThirdParty/ardupilot/Tools/autotest/default_params

# Loop through the number of UAVs
for ((i=0; i<$num_uavs; i++)); do
    # Copy gazebo-iris.parm to gazebo-uav{i}.parm
    cp gazebo-iris.parm gazebo-uav$i.parm

    # Append SYSID_THISMAV to the copied parameter file
    echo "SYSID_THISMAV $((i+1))" >> gazebo-uav$i.parm
done

# Verbose
echo "Done adding parameter files for $num_uavs UAVs"
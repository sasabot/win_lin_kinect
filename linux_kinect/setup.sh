#!/bin/bash

# check path

if [[ $(pwd | awk -F/ '{print $NF}') != "linux_kinect" ]]
then
    echo "setup must be conducted under linux_kinect."
    exit
fi

# check settings

cmake_file="CMakeLists.txt"

if [[ $1 == "--lib" ]]
then
    cp ".templates/CMakeLists.lib" $cmake_file
elif [[ $1 == "--all" ]]
then
    cp ".templates/CMakeLists.all" $cmake_file
else
    echo "options: --lib or --all"
    exit
fi

tab2=$'  '

# setup msgs

msg_dir="msg"
ls_msg=$(ls $msg_dir | tr '\n' ' ')
num_of_msgs=$(ls $msg_dir | wc -l)

for (( num=1; num<=$num_of_msgs; num++))
do
    target=$(echo $ls_msg | awk '{print $'$num'}')
    check_if_exists=$(grep "${target}" $cmake_file)
    if [[ "$check_if_exists" == "" ]]
    then
        write_to_line=$(grep -n -m 1 "auto-add messages" $cmake_file | cut -d ':' -f1)
        write_to_line=$(($write_to_line + 3))
        echo "${tab2}${tab2}${target}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
    fi
done

# setup srvs

srv_dir="srv"

ls_srv=$(ls $srv_dir | tr '\n' ' ')
num_of_srvs=$(ls $srv_dir | wc -l)

for (( num=1; num<=$num_of_srvs; num++))
do
    target=$(echo $ls_srv | awk '{print $'$num'}')
    check_if_exists=$(grep "${target}" $cmake_file)
    if [[ "$check_if_exists" == "" ]]
    then
        sed -i "s/set(GENERATE_SRV)/set(GENERATE_SRV 1)/g" $cmake_file
        write_to_line=$(grep -n -m 1 "auto-add services" $cmake_file | cut -d ':' -f1)
        write_to_line=$(($write_to_line + 3))
        echo "${tab2}${tab2}${target}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
    fi
done

catkin bt

#!/bin/bash

# ex.: . fw_release.sh

parser_version() {
    local parser_str=$1
    local grep_str=`grep $parser_str main/include/util.h`
    local chrlen=${#grep_str}
    local ver_num=`grep $parser_str main/include/util.h | cut -c $chrlen`
    return $ver_num
}

get_version() {
    parser_version "VERSION_MAJOR"
    local maj_ver=$?
    parser_version "VERSION_MINOR"
    local min_ver=$?
    parser_version "VERSION_PATCH"
    local pat_ver=$?

    local ver=v$maj_ver.$min_ver.$pat_ver
    echo $ver
}

main() {
    echo "fw release start"
    now="$(date +'%Y%m%d%H%M')"

    ver_str=$(get_version)
    printf "ver_str: $ver_str\n"
    
    testing_fw_path=./release/testing_fw/$ver_str/$now/
    mkdir -p $testing_fw_path

    shipping_fw_path=./release/shipping_fw/$ver_str/$now/
    mkdir -p $shipping_fw_path

    # testing fw
    make
    cp build/bootloader/bootloader.bin $testing_fw_path/.
    cp build/lulupet_fw.bin $testing_fw_path/.
    cp build/ota_data_initial.bin $testing_fw_path/.
    cp build/partitions.bin $testing_fw_path/.
    
    # shipping fw
    sed -i 's/FUNC_TESTING_FW 1/FUNC_TESTING_FW 0/g' main/include/util.h
    make
    cp build/bootloader/bootloader.bin $shipping_fw_path/.
    cp build/lulupet_fw.bin $shipping_fw_path/.
    cp build/ota_data_initial.bin $shipping_fw_path/.
    cp build/partitions.bin $shipping_fw_path/.

    sed -i 's/FUNC_TESTING_FW 0/FUNC_TESTING_FW 1/g' main/include/util.h
}

main $*

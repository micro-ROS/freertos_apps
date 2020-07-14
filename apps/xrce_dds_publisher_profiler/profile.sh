
EXTENSIONS_DIR=/workspaces/foxy_ws/uros_freertos_olimex/firmware/freertos_apps/microros_olimex_e407_extensions
APP_DIR=$(pwd)
BASE_BSS=88488 # data:156 + bss:88332


for REF_XML in XML REF; do
    for RELIABILITY in RELIABLE BEST_EFFORT; do
        echo "#pub topic_size bss+data stack" > results-$REF_XML-$RELIABILITY.txt

        for NUM_PUB in 1 5 10 20; do 
            for SIZE_TOPIC in 1 342 683 1025 1366 1707 2048 2390 2731 3072; do 
                echo "#define PUB_NUM $NUM_PUB" > config.h
                echo "#define TOPIC_SIZE_M $SIZE_TOPIC" >> config.h
                echo "#define UCLIENT_PROFILING_$REF_XML" >> config.h
                echo "#define UCLIENT_PROFILING_$RELIABILITY" >> config.h
                pushd $EXTENSIONS_DIR >/dev/null   
                    rm build/micro-ROS.elf
                    echo "Building pubs: $NUM_PUB topic: $SIZE_TOPIC B - MODE: $REF_XML and $RELIABILITY"
                    make UROS_APP_FOLDER=$APP_DIR 2> /dev/null > /dev/null
                    DATA_USAGE=$(size -t build/micro-ROS.elf |grep TOTALS | awk -F ' ' '{print $2}')
                    BSS_USAGE=$(size -t build/micro-ROS.elf |grep TOTALS | awk -F ' ' '{print $3}')
                    BSS_USAGE=$((BSS_USAGE + DATA_USAGE - BASE_BSS))
                    echo "Flashing"
                    openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase build/micro-ROS.bin 0x08000000" -c "reset" -c "exit" 2> /dev/null > /dev/null
                    echo "Running"
                    STACK_USAGE=$(sudo python3 -c "import serial; ser=serial.Serial('/dev/ttyUSB1',115200); print(ser.readline().decode('utf-8').strip())")
                    echo "Static memory usage: $BSS_USAGE Stack peak: $STACK_USAGE"
                popd >/dev/null
                echo $NUM_PUB $SIZE_TOPIC $BSS_USAGE $STACK_USAGE >> results-$REF_XML-$RELIABILITY.txt
            done
        done
    done
done





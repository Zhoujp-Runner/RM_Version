#!/bin/bash 

sec=1
cnt=0
name=TUP-Vision
cd /home/liubiao/
sudo chmod 777 ./$name
# cd //home/liubiao/$name/build/
sudo make clean && make -j8 
while [ 1 ] 
do
    count=`ps -ef | grep $name | grep -v "grep" | wc -l`
    echo "Thread count: $count" 
    echo "Expection count: $cnt" 
    # cd /home/liubiao/$name/build/
    # (sudo chmod 777 /dev/ttyACM0 || chmod 777 /dev/ttyACM1)
    if [ $count -ge 1 ]; then
        echo "The $name is still alive!" 
        sleep $sec
    else
        echo "Starting $name..." 
        cd /home/liubiao/$name/build/
        ./$name 
        echo "$name has started!"   
        sleep $sec
        # ((cnt=cnt+1))
        # if [ $cnt -gt 9 ]; then
        #     reboot
        # fi
    fi
done




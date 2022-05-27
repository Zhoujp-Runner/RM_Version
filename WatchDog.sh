#!/bin/bash 

# source /opt/intel/openvino_2021/bin/setupvars.sh
sec=1 
cnt=0 

name=Adjust_Version         #TODO:程序所在文件夹名称
program_name=Adjust_Version #TODO:项目名称

echo "liubiao.2021" | sudo -S chmod 777 /home/liubiao/Desktop/$name #-S --stdin 重定向输入,避免手动输入密码

cd /home/liubiao/Desktop/$name/build/   #TODO:程序build路径
#make clean && 
make -j6 

while [ 1 ] 
do 
    count=`ps -ef | grep $name | grep -v "grep" | wc -l`

    #查看串口是否开启（未开启则等待开启）
    while true ; do
        if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ]]; then
            break
        else
            continue
        fi
    done

    # if [ $count -eq 0 ]; then 
    #     echo "start $program_name"
    #     ./$program_name 
    # fi
    echo "Thread count: $count" 
    echo "Expection count: $cnt" 

    (sudo chmod 777 /dev/ttyACM0 || chomd 777 /dev/ttyACM1) #为串口赋予权限

    if [ $count -gt 2 ]; then 
        echo "The $name is still alive!" 
        sleep $sec 
    else  
        echo "Starting $name..." 
        # gnome-terminal -- bash -c "cd /home/tup/Desktop/$name/build/;
        # ./$name;exec bash;" 
        cd /home/tup/Desktop/$name/build/   #TODO:程序build路径
        make -j6 && ./$program_name
        echo "$name has started!"   
        sleep $sec 
        ((cnt=cnt+1)) 
        if [ $cnt -gt 9 ]; then 
            #reboot              #比赛前打开
            echo "reboot!"       #调试时打开
        fi 
    fi 
done





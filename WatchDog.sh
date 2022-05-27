#!/bin/bash 

<<<<<<< HEAD
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
=======
sec=1
cnt=0
name=Adjust_Version         #TODO:程序所在文件夹名称
cd /home/liubiao/Desktop/   #TODO:程序所在文件夹路径

echo "password" | sudo -S chmod 777 $name   # -S --stdin 直接从标准输入读取密码，不需要手动添加
                                            # TODO:password 为自己的密码
# cd $name/build/
cmake .. && make clean && 
make -j8 

while [ 1 ] 
do
    count=`ps -ef | grep $name | grep -v "grep" | wc -l`    # 查看进程数
    # if [ $count -eq 0 ]
    # then
    #     echo "start $name"
    #     ./$name
    # fi
    echo "Thread count: $count" 
    echo "Expection count: $cnt" 
    # cd /home/liubiao/$name/build/

    #检查串口是否连接（未打开则等待）
    while true ; do
        if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ]] ; then
            echo "tty is connected"
            break
        else
            #等待串口连接
            echo "tty is not connected"
            sleep 1
        fi
    done
    (sudo chmod 777 /dev/ttyACM0 || chmod 777 /dev/ttyACM1)  # 给串口权限
    
    if [ $count -gt 2 ]; then
<<<<<<< HEAD
=======
>>>>>>> main
>>>>>>> master
        echo "The $name is still alive!" 
        sleep $sec 
    else  
        echo "Starting $name..." 
        # gnome-terminal -- bash -c "cd /home/tup/Desktop/$name/build/;
        # ./$name;exec bash;" 
<<<<<<< HEAD
        cd /home/liubiao/Desktop/$name/build/   #TODO:程序build文件夹路径
        make -j8 && ./$name
        echo "$name has started!"   
=======
<<<<<<< HEAD
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




=======
        cd /home/liubiao/Desktop/$name/build/   #TODO:程序build文件夹路径
        make -j8 && ./$name
        echo "$name has started!"   
>>>>>>> master
        sleep $sec
        ((cnt=cnt+1))
        if [ $cnt -gt 9 ]; then
            # reboot        #上赛场前打开
            echo "reboot!"  #调试使用
        fi
    fi
<<<<<<< HEAD
done
=======
done
>>>>>>> main
>>>>>>> master

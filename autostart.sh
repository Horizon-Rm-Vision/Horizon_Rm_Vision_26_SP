#!/bin/bash 
#V1版自启动脚本
#部署程序自启动方法：在“启动应用程序”中添加启动项，名字随便
#启动项命令为：gnome-terminal  --  bash    /home/horizon/Horizon_Rm_Vision_26_SP/autostart_V1.sh
#NX风扇满速一直转
echo '1' | sudo -S jetson_clocks --fan
sec=2
cnt=0
name=horizon/work/Horizon_Rm_Vision_26_SP
program_name=auto_aim_debug_mpc
cd /home/$name/build/

while [ 1 ]
do
    # 检测进程数量
    count=$(pgrep -c $program_name)
    echo "线程数量: $count"
    echo "期望数量: $cnt"
    if [ $count -gt 0 ]; then
        echo "$name 仍在运行!"
        sleep $sec
    else
        echo "启动 $name..."
        # 检查是否存在互斥锁
        if [ ! -f /tmp/$program_name.lock ]; then
            touch /tmp/$program_name.lock
            cd /home/$name/build/
            #手动查询串口设备代号命令：ls /dev/ttyS* -alt
            #英雄，哨兵和串口版步兵用ttyACM0
            echo '1' | sudo -S chmod 666 /dev/ttyACM0
            echo '1' | sudo -S chmod 666 /dev/ttyACM1
            #老串口模块
            echo '1' | sudo -S chmod 666 /dev/ttyUSB0
            echo '1' | sudo -S chmod 666 /dev/ttyUSB1
            #飞机的达妙NX新载板用ttyTHS0
            echo '1' | sudo -S chmod 666 /dev/ttyTHS0
            echo '1' | sudo -S chmod 666 /dev/ttyTHS1
            #jetson nano用ttyCH341USB0
            echo '1' | sudo -S chmod 666 /dev/ttyCH341USB0
            echo '1' | sudo -S chmod 666 /dev/ttyCH341USB1

            ./$program_name
           
            rm /tmp/$program_name.lock
            echo "$name 已启动!"
            ((cnt=cnt+1))
        else
            echo "另一个 $program_name 实例正在运行."
        fi
        sleep $sec
        if [ $cnt -gt 9 ]; then
            echo "重启!"
            #reboot
        fi
    fi
done
>>>>>>> b41fc86a94dcabff32797a6432231a7ce0b99adf

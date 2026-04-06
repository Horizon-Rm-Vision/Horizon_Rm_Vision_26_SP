#infantry分支开发日志
###A2
-根版本：main-M4
-功能更新：添加ROS接口，方便进行仿真测试（用到ROS接口的源文件CmakeLists.txt需更改，ROS/src目录下有CMAKELISTS,可以参考更改）
-已知问题：通过ROS接口传进去的图像帧率低，可能与ROS节点接收图像的方式有关。

###A3 -3_9
-根版本：M6-DEV3
-更新功能：通过yaml控制是否锁中心，涉及到更改的文件 planner.c.h solver.c.h
主文件的画中心功能
    if (planner.aim_center_) {
      auto center_image_points = solver.reproject_point(planner.center_points);
      tools::draw_points(img, center_image_points, {255, 0, 0}, 5);
    }


###A4 -3_9
-修复：1.锁定中心改为装甲板位置，增加英雄弹道解算
      2.主文件的中心坐标重投影坐标原来为私有属性，改为共有便于访问
      经测试可以正确运行投影出中心坐标，弹道解算加上了预测时间，英雄弹道解算是否改对待验证。


###A5 -4_6
-增加：1.对前哨战高低差建模，x 11维状态量的后两位长短轴和高低差，在目标为前哨战时被夺舍为另两块装甲板与第一块装甲板的高度差，
      outpost_z01 outpost_z02，
      取第一次见到的前哨站装甲板高度为旋转中心高度。
      涉及到的更改只有target.cpp/.hpp tracker.cpp，另：注意planner的锁中心机制会涉及到修改速度噪声方差，
      可以选择在planner的锁中心里注释掉或者把double v1, v2 在头文件里声明为public。
      
      
      
      gimbal现在是没有速度通信位的老版本（为兼容云台模拟器）
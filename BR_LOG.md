#infantry分支开发日志
###A2
-根版本：main-M4
-功能更新：添加ROS接口，方便进行仿真测试（用到ROS接口的源文件CmakeLists.txt需更改，ROS/src目录下有CMAKELISTS,可以参考更改）
-已知问题：通过ROS接口传进去的图像帧率低，可能与ROS节点接收图像的方式有关。

###A3 -3_9
-更新功能：通过yaml控制是否锁中心，涉及到更改的文件 planner.c.h solver.c.h
主文件的画中心功能
    if (planner.aim_center_) {
      auto center_image_points = solver.reproject_point(planner.center_points);
      tools::draw_points(img, center_image_points, {255, 0, 0}, 5);
    }


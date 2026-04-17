# Horizon_Rm_Vision_26_SP

## main 分支开发日志

##### by ZYL

#### MileStone 1：

- **功能更新:**基于REV4,修复了大恒相机的曝光问题，为yolov5模式新增了TensorRT10推理框架的支持，使其支持在Jetapck6使用TensorRT10加速推理以及带NVIDIA独显的设备使用TensorRT10加速推理，是正式准备测试的第一个版本。

#### MileStone 2：

- **功能更新：**根据io/ros2部分代码逆转写出的sp_msgs通信包，恢复了其和ros2程序通信的能力

#### MileStone 3：

- **功能更新:**修改了主CMakeLists使其按条件编译，默认不编译全部组件，以加快编译速度，具体组件编译条件详细见上面的代码编译部分，修改通信协议，收发单位由弧度改为角度，
- **质量更新:**修正大恒相机对arm64的兼容问题

#### MileStone 4:

- **功能更新:**修改通信默认模式0为自瞄模式，合并UI窗口，补充UI信息绘制，修改文件读取机制，从该版本开始不再需要从根目录启动程序，可以直接在build启动,TRT模式性能优化，将大量OpenCV任务放到GPU上，TensorRT模式下的性能提升近60%
- **质量更新:**修复拆数的bug,引入新版大恒arm64的lib以修复部分arm设备的编译问题

#### MileStone 5：

- **功能更新:**
  1.删除trt模式的int8校准器；正式引入V7版通信模拟器;
  2.手眼标定程序C板通信替换为串口通信模式;标定板由圆点标定板改为棋盘标定板（待实车测试）
  3.迈德相机去除gamma调参;
  4.增加曲线绘制功能(from infantryA);
  5.引入带空气阻力的弹道结算方法（移植自华科英雄24开源），使用yaml选择是否启用，运行demo未发现问题，待实车测试
  6.CMakelists引入全局宏控制写法，现可以使用主CMakelists控制编译时是否使用带角速度收发的通信模式（对应代码内的宏SR_VEL），以及CMakelists可直接控制是否启用TRT推理模式
  7.sp_msgs更新为新哨兵联合通信版，需安装ros2的geometry_msgs（命令：sudo apt install ros-humble-geometry-msgs），引入新版哨兵导航联合通信机制（来自sentry）；
- **质量更新:**
  1.demo数据回归;补充部分相机内参;
  2.修复通信和UI显示的数据问题,终端内容数据更新
  3.迈德相机颜色修正,修正手眼标定缺失的函数
  4.引入cuda算子，大量重写trt部分，性能优化，3050测试机上单张推理平均时间由5.5ms提升至4.0ms,并进一步改进尝试，收益不明显；
  5.内置v5模型的原版onnx（yolov5a-0708.onnx）、Jetson NANO Orin（yolov5a-fp16-orin.engine）和40系显卡的fp16 engine模型文件（yolov5a-fp16-ada.engine），原来的30系显卡的fp16 engine模型改名为yolov5a-fp16-ampere.engine
  6.UI绘制数值单位由弧度修改为角度;
- **待验证:**
  1.修改前哨战机制（from infantryA），如果可以正常使用，再修改观测数据更新卡尔曼滤波（目前是一直用中间装甲板的高度数据更新，应该是每块装甲板的高度数据都能更新）。
  2.为解决当前的异常数值问题暂时采用一个简单过滤机制（对应代码内的宏EZ_FILTER），机制详细实现见参数文件内的解释（待实车测试）

#### MileStone 6：

- **功能更新:**
  1.新增自适应串口名称模式，记录常用的串口名，在yaml的串口名称设置为“auto”则可启用自适应串口名功能，不使用“auto”仍然可以指定某个单一名称的串口,扩展了自动串口名的列表，将后缀扩展到4，新增ttyCH341USBX,并修改探测顺序
  2.集成英雄/飞机/全向最新标定参数
  3.新增按距离远近锁中心机制,(由宏AIM_CENTER启用)当地方距离足够远时不再追踪单板而是追踪车中心轴(from infantry)
- **质量更新:**
  1.修复了部分手眼标定程序仍然使用需要圆点标定板的问题以及启动问题，已全部棋盘格化,并新增手眼标定方法指南
  2.串口通信模拟器收发频率提升至1000Hz
  3.trt模式隐藏原始窗口提升在nano上的帧率
  4.将DM_IMU的代码加入LIM_CODE编译限制，屏蔽其依赖库serial文件夹(部分电脑需要执行sudo apt install libserial-dev后才能正常编译)，并将与其相关的mt_standard、standard_mpc、uav、uav_debug、dm_test、minium_vision_system在cmake中注释禁止编译
  5.将串口读取数据机制由单包处理改为多包处理,更改串口数据循环读取逻辑,删除无用的死循环机制,以修正数据滞后导致的云台反应不稳定问题
  6.修复哨兵模式下无发正常编译标定采集程序的问题
- **已验证:**
  1.移植华科的英雄弹道解算经测试暂未发现问题，且在3-5m测试时英雄大弹丸弹道解算表现优于同济原版
  2.EZ_FILTER已移除
- **待验证:**
  1.基于peek2的四元数取数方式,待多几个版本测试以验证可靠性(from infantry)
## 

#### MileStone 7：

- **功能更新:**
  1.统一UI架构,统一standard_serial和auto_aim_debug_mpc的ui显示,合并img_tools和新增UI绘制的全部内容,支持现有src和test调用,支持使用yaml控制UI绘制与否和imshow启用与否
  2.合并哨兵模式到auto_aim_debug_mpc,为standard_serial引入哨兵模式
  3.UI显示新增哨兵导航内容(from sentry)
  4.添加开火约束(由宏FIRE_CONSTRAINT启用),分为角度约束和距离约束
  5.SPSREMU更新V8版本,支持扩展通信,详细使用方法见py文件注释
  6.新增自启脚本,分为无ROS依赖的V1和有ROS依赖的V2
  7.aimer模式的反陀螺判定参数引出2rad/s,delta_angle,加入当前敌车是否小陀螺的ui显示
  8.新增能量机关识别测试用yaml(auto_buff_test)
  9.合并哨兵通信协议修改至UL版本,修改长度和使用浮点数收发弹速,新增form信息(from sentry)
  10.串口收发弹速采用将电控发送时乘10的数值接收时除以10的方法获得一位浮点数
- **质量更新:**
  1.standard_serial无法开火问题修复
  2.锁中心机制修改(from infantryA)
  3.修正统一ui显示的在mpc模式的armor_x/y/z问题
  4.修正串口通信SR_VEL和SENTRY_MODE的兼容问题,现在两者可同时启动
  5.合并完整形态和RMUL以来新标定的内外参
- **已验证:**
  1.基于peek2的四元数取数方式,经测试已验证其可靠(from infantry)
- **待验证:**
  1.aimer模式下英雄弹道解算可能存在问题,待测试验证
## 

#### Release Candidate 1 DEV5：

- **功能更新:**
  1.集成两种新版前哨站识别方案,使用宏NOVA_OUTPOST_V1/2启用,待测试验证可行性
  2.SPSREMU升级至V9,支持新版统一通信
- **待验证:**
  1.疑似daheng相机帧率不稳定引起云台不稳定,暂时修复待验证
## 
待开发任务：
1.TRT推理性能进一步优化
2.NANO简化代码模式（对应代码内的宏LIM_CODE），启用后会去掉一些暂时不必要的代码，用于提升编译速度和开销（尤其是JETSON）,目前已将src里不需要的几个程序和DM_IMU部分的代码屏蔽
3.分析bullet_count对火控的影响
4.新版前哨站机制
5.打符子程序
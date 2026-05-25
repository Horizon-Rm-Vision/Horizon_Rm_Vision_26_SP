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

#### Release Candidate 1 DEV8：

- **功能更新:**
  1.新增WEB调试接收器,支持将ui_manager和plotter的所有数据通过web发出,在接收端利用收到的数据重绘,具体使用方法见web_ui_receiver的readme.md
- **已验证:**
  1.已确认daheng相机帧率不稳定引起云台不稳定的问题并不存在
## 

#### Release Candidate 1 DEV9：

- **功能更新:**
  1.哨兵通信更新
- **已验证:**
  1.已确认daheng相机帧率不稳定引起云台不稳定的问题并不存在
## 
#### Release Candidate 1 DEV10：

- **功能更新:**
  1.新增基于YOLOX和传统视觉修正的能量机关识别模型,支持OpenVINO模式与TensorRT模式运行,有aimer(auto_buff_debug_mpc)和mpc(auto_buff_debug_serial)两种模式,demo则启动auto_buff_test即可

## 

#### Release Candidate 1 DEV11：

- **功能更新:**
  1.能量机关识别根据26赛季规则进行修改,大符模式新增双板选择功能,按选择离图像中心最近的策略选板
  2.测试启用内录模式

## 

#### Release Candidate 1 DEV12：

- **功能更新:**
  1.为auto_buff_debug_mpc和auto_buff_debug_serial引入ui_manager调试信息,同步支持web_receiver显示
  2.正式为auto_aim_debug_mpc的方法为standard_serial,auto_buff_debug_mpc,auto_buff_debug_serial添加内录模式,可以将本次运行的视频连带收到的电控数据录制下来,可以使用test程序重新解算回放
  3.弹速机制更新,yaml设为auto时保持接受电控弹速,设为数值时使用赋予的数值
- **质量更新:**
  1.自启脚本和gimbal已删除已经不再使用的ths串口

## 

#### Release Candidate 1 DEV13：

- **功能更新:**
  1.能量机关算法改进,引入buff_tracker,使用FSM状态机,连续检测确认才进入TRACKING,超过时间阈值自动回退LOST,观测选择使用多候选+马氏距离门控,避免错误匹配,新增运动补偿PoseBuffer对云台姿态进行延时插值

## 

#### Release Candidate 1 DEV14：

- **功能更新:**
  1.能量机关算法改进,修改扇叶id获取方式,修改成类似WUST的方法,多扇叶同时进行ekf更新,每个扇叶独立分配 ID,贪心去重避免 ID 冲突,PnP 与 EKF 解耦

## 

#### Release Candidate 1 DEV15：

- **功能更新:**
  1.能量机关算法改进,4维马氏距离匹配,协方差自适应门控,解决了过0时id可能误匹配的问题

## 

#### Release Candidate 1 DEV16：

- **功能更新:**
  1.能量机关算法改进,补齐了部分角度跳变缺少的limit_rad,增加了roll残差门限
  2.R标传统识别ui移到右下角,能量机关相关ui排版优化

## 

#### Release Candidate 1 DEV17：

- **功能更新:**
  1.引入中南的lenet传统视觉装甲板识别器,yolo_name设置为tra即可启用
  2.剥离OpenVINO,现在可以单独启用和关闭OpenVINO和TensorRT,实现可选无OpenVINO依赖,实现Jetson纯粹的TensorRT模式

## 

#### Release Candidate 1 DEV18：

- **功能更新:**
  1.NOVA_OUTPOST_V2机制更新,修法NIS无条件运行和temp_lost未即使重置计数的问题
  2.全向感知相机扩展,引入odin1相机支持,现可以接收odin1的官方ROS驱动发送的图像进行全向感知,可以设置缩放分辨率以降低开销,可与普通USB相机共存,并可以单独运行test程序进行图像收发测试
  3.将原带全向感知的sentry系程序(sentry_bp.cpp sentry_debug.cpp sentry_multithread.cpp sentry.cpp)改为串口通信,并基于sentry.cpp编写sentry_odin.cpp,支持调用odin1进行全向感知并设置相对主相机的位置
  4.SPSREMU更新,支持最新哨兵通信协议

## 

#### Release Candidate 1 DEV19：

- **功能更新:**
  1.web_ui_receiver支持收发原始图像
  2.plotter支持绘制多坐标轴子图曲线

## 

#### Release Candidate 1 DEV20：

- **功能更新:**
  1.plotter的web发送相关设置合并到ui设置中
  2.删除旧版锁中心,引入新版锁中心机制(当到达旋转阈值时，不再瞄准单个装甲板，而是瞄准整车旋转中心，计算正对相机的那个面进行射击),同时适用于aimer模式和mpc模式,进入时会接管开火
  3.实验性的为新前哨站也引入wmj式锁中心机制,使用yaml单独控制启用与否
  4.前哨站锁中心模式下机制改进:进入条件改为依赖收敛而非转速阈值进入锁中心,设置独立开火容差,三级转速钳位

## 

#### Release Candidate 1 DEV21：

- **功能更新:**
  1.引入华科的yolox装甲板模型,支持ov和trt模式,支持原版的svm标签修正,并扩展支持了lenet和resnet模型进行标签修正,并增加了传统修正支持
  2.统一了所有装甲板模型的语义,基于新版规则,只有1号装甲板允许为大装甲板,其余包括基地全为小装甲板
  3.合并了NOVA_OUTPOST_V1和V2

## 

#### Release Candidate 1 DEV22：

- **功能更新:**
  1.buff机制更新,yolox模式过滤已激活的扇叶
  2.standard_serial改名为auto_aim_debug_aimer,auto_buff_debug_serial改名为auto_buff_debug_aimer
  3.为auto_buff_debug_mpc和auto_buff_debug_aimer引入完整的新版大符支持,使用串口收发大小符模式切换,ui同步更新
  4.为auto_buff_test引入完整新版大符支持,使用yaml控制模式
  5.全新auto_vision_mpc模式,支持步兵/哨兵通过串口收发动态切换mpc模式的自瞄/大符/小符,为步兵/哨兵上场的标准程序
  6.SPSREMU升级至v10版本,支持制定发送模式,支持按周期切换发送模式
  7.cmakelists为能量机关所用识别机制引入独立开关(BUILD_BUFF),支持英雄/飞机禁用能量机关识别代码加快编译速度
  8.修复了buff的yolox模型和aim的yolox模型重名的问题

## 

#### Release Candidate 1 DEV23：

- **功能更新:**
  1.将io的ros部分和全向感知部分(包括odin驱动)捆绑BUILD_SENTRY,以减少其他兵种的编译开销
  2.新增一个option "BUILD_OMNI",为全向感知(包括odin驱动)再加一层限制,只有启用哨兵模式,启用BUILD_OMNI才启用功能并编译
  3.为各个车型不同配置和组件需求编写各自的一键编译脚本,部署到具体车时无需再手动修改cmakelists
  4.改进程序读取相机内外参的方式,内外参可以不内置在各个程序启动的yaml内部,而是读取camera_param_path所对应的独立相机内外参的yaml文件,当然如果没有设置camera_param_path或者地址无效则会回退回去直接读取主yaml内置的参数
  5.改进auto_vision_mpc在jetson上的性能问题
  6.修改web_ui_receiver的发送端口为9878以解决与官方图传的端口抢占问题
  7.删除面向前哨站的锁中心机制,修复自DEV20以来前哨站存在的问题
## 

#### Release Candidate 1 DEV24：

- **功能更新:**
  1.为飞机引入基于角度的顶部前哨站装甲板过滤机制,可由yaml启禁用和调参
  2.修复了yolox模式下lenet和resnet标签表映射异常导致的阻断识别的问题
  3.修复yolox模式下原始角点周期性出现的异常角点排序引起的原始角点异常严重依赖传统修正并引起识别过程中目标值异常变化的问题
  4.合并yolox两个模式ov/trt的专属参数,清除无效参数use_svm
  5.屏蔽standard的编译,修改aim_vision_mpc的编译逻辑,只有启用buff才编译
  6.同步哨兵新版通信
  7.终端默认不再输出收到的原始数据
  8.集成RC1-DEV1以来所有新增模式用于的jetson的engine文件
  9.所有src层添加终端输出fps
## 

#### Release Candidate 1 DEV25：

- **功能更新:**
  1.全新大符机制,基于简化状态机的双板检测,支持双扇叶独立EKF(宏BIG_BUFF_DUO_EKF),大符开火机制修正(宏BIG_BUFF_FIRE_FIX)
  2.大符引入与装甲板类似颜色设置机制,新版大符机制相关参数由yaml控制
  3.编译脚本编译完成后自动进入build文件夹
  4.开火约束机制更新,不再以绝对角度而是相对角度约束开火
  5.auto_aim_test和auto_buff_test增加对应aimer模式的ui
## 

#### Release Candidate 1 STABLE：

- **功能更新:**
  1.集成RC1-DEV1至DEV25全部更新内容
  2.SPSREMU支持最新版哨兵通信格式
  3.能量机关识别支持通过YAML设置R标修正ROI大小
  4.为英雄增加过滤最高处前哨站装甲板(宏HERO_OUTPOST_FILTER),使其只击打下面两块
  5.哨兵自启增加锁核机制
- **质量更新:**
  1.合并了能量机关识别yolox模式冗余重复的传统修正函数
  2.能量机关识别全部UI绘制全部由UI_MANAGER托管
  3.集成目前全部车型内外参文件
  4.recorder的默认保存位置移出build防止误删
  5.修复新recorder机制的导致aimer模式回放机制的问题
- **待测试:**
  1.MPC模式的回放程序(auto_aim_test_mpc),可用性待测试
  2.由于mpc机制的特殊性,recorder原有录制内容无法满足新回放模式运行,现已修改recorder以适配该新模式,保留对aimer的旧回放程序的支持,可行性待测试
## 

待开发任务：
1.基地灯识别
2.集成英雄自定义客户端图传编码端


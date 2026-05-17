# Horizon_Rm_Vision_26_SP编译环境配置文档

**by ZYL**

## 模式选择

| 硬件                      | OpenVINO CPU模式 | OpenVINO GPU模式 | TensorRT 10模式 |
| ------------------------- | ---------------- | ---------------- | --------------- |
| Intel CPU                 | 支持             | 支持             | 不支持          |
| Intel CPU with Nvidia GPU | 支持             | 支持             | 支持            |
| AMD CPU                   | 支持             | 支持             | 不支持          |
| AMD CPU with Nvidia GPU   | 支持             | 不支持           | 支持            |
| Jetson Orin               | 支持             | 不支持           | 支持            |
| 其他ARM64开发板           | 支持             | 不支持           | 不支持          |

**注意:**哨兵联合通信模式需要安装ROS2-Humble和sp_msgs

## 环境配置

#### 1.相机驱动

##### 需要迈德/大恒/海康驱动,SDK文件从官网下载即可,当然不同品牌的驱动安装方法不同：

```
大恒驱动：要使用：sudo chmod a+x 文件名，处理后：./文件名 运行
迈德驱动：直接运行解压目录下安装脚本：sudo ./install.sh
海康驱动:解压压缩包安装对应cpu架构的deb版(PC用x86_64,NX/AGX用aarch64)
```

#### 2. 基础依赖

**大部分都可以直接apt安装,命令:**

```
sudo apt install -y \
    git \
    g++ \
    cmake \
    can-utils \
    libopencv-dev \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    openssh-server \
    screen
```

ceres没有apt版,需要源码安装,依赖eigen上面的apt命令已经包含了,直接解压ceres2.20源码包(https://github.com/ceres-solver/ceres-solver/releases/tag/2.2.0),按普通cmake程序编译安装即可,命令:

```
cmake ..
make -j16 #注意线程数不要太多,ceres编译吃内存,容易爆内存卡死系统
sudo make install
```

#### 3.OpenVINO(必装)

无论哪种设备目前都必须安装OpenVINO,本项目推荐使用2024.6.0版,具体安装方法见我的另外两篇文章<<OpenVINO配置教程(X86版)>>和<<OpenVINO配置教程(ARM64版)>>

#### 4.CUDA&CUDNN&TensorRT(可选)

有Nvidia独显的PC支持使用TensorRT加速推理,本程序所使用的TensorRT版本为10,其依赖于CUDA11/12,CUNDNN8/9,具体安装方法见另一篇文章<<Ubuntu CUDA CUDNN TRT 配置教程>>

#### 5.OpenCV with CUDA(可选)

有Nvidia独显的PC支持使用TensorRT加速推理,本程序使用的TensorRT推理框架依赖OpenCV的CUDA扩展组件,所以要启用TensorRT模式必须带CUDA扩展的OpenCV,而OpenVINO模式只需要普通版的OpenCV,具体安装方法见另一篇文章<<Ubuntu OpenCV4 配置教程>>

#### 6.ROS2&sp_msgs(可选)

哨兵模式需要ROS2-humble和sp_msgs,ROS2安装方法这里不再赘述,需要注意的是使用fishros一键安装的ROS2可能缺一些包,而该程序对ROS的需求仅在io部分，其中ament_cmake、rclcpp、std_msgs、rosidl_typesupport_cpp均为ROS的标准包，使用apt即可下载补全,命令:

```
sudo apt install ros-humble-ament-cmake
sudo apt install ros-humble-rclcpp
sudo apt install ros-humble-std-msgs
sudo apt install ros-humble-rosidl-typesupport-cpp
```

而sp_msgs是本程序自定义的通信数据包,需要自行编译调用,详细使用方法见"sp_msgs"文件夹下的readme.md

## 编译运行

#### 1.修改CMakeLists

打开代码根目录的CMakelists,根据硬件配置和依赖安装情况,选择是否启用使用CUDA/TensorRT和OpenVINO,是否使用哨兵模式(宏BUILD_SENTRY),以及是否编译其他组件(test工具,标定工具等)
注意:修改主cmakelists后需要删除build文件夹下的所有文件重新执行cmake ..命令,否则修改不会生效
如需使用TensorRT模式则还需要打开根目录下的cmake文件夹下的"FindTensorRT.cmake",修改第31行的TensorRT位置和版本

#### 2. 编译程序

**在代码根目录创建build文件夹,进入build文件夹,执行以下命令,注意:**

```
cmake ..
make -j12
```

#### 3.运行demo

从根目录进入configs文件夹,打开demo.yaml,修改"yolo_name":
如果要使用OpenVINO模式,则改为"yolov5_ov",接着修改"device",如果使用纯CPU改为"CPU",如果要使用GPU则改为"GPU";
如果要使用TensorRT模式,则改为"yolov5_trt",接着修改"yolov5_trt_model_path",根据你的GPU类型进行选择,本程序只集成了30系(ampere),40系(ada)和Jetson Orin(orin)的engine文件,如果是50系卡(blackwell),则需要手动转换模型,搜索TensorRT安装目录,找到trtexec,使用trtexec将assets下的yolov5a-0708.onnx转换为engine模型,建议量化精度为fp16,示例命令:

```
trtexec --onnx=yolov5a-0708.onnx --fp16 --saveEngine=yolov5a-fp16-blackwell.engine
```

转换后将得到的模型放到assets文件夹下,修改yolov5_trt_model_path为新模型的位置即可

修改并保存后,在build文件夹下打开终端,使用命令启动demo:

```
./auto_aim_test
```


URDF 不能单独使用，需要结合 Rviz 或 Gazebo，URDF 只是一个文件，需要在 Rviz 或 Gazebo 中渲染成图形化的机器人模型

准备:新建功能包，导入依赖

核心:编写 urdf 文件

核心:在 launch 文件集成 URDF 与 Rviz

在 Rviz 中显示机器人模型萨

在 ROS 中，可以将 urdf 文件的路径设置到参数服务器，使用的参数名是:robot_description

重复启动launch文件时，Rviz 之前的组件配置信息不会自动保存，需要重复执行步骤4的操作，为了方便使用，可以使用如下方式优化:

首先，将当前配置保存进config目录然后，launch文件中 Rviz 的启动配置添加参数:args,值设置为-d 配置文件路径


## URDF

URDF 文件是一个标准的 XML 文件，在 ROS 中预定义了一系列的标签用于描述机器人模型，机器人模型可能较为复杂，但是 ROS 的 URDF 中机器人的组成却是较为简单，可以主要简化为两部分:连杆(link标签) 与 关节(joint标签)，接下来我们就通过案例了解一下 URDF 中的不同标签:

robot 根标签，类似于 launch文件中的launch标签
link 连杆标签
joint 关节标签
gazebo 集成gazebo需要使用的标签

关于gazebo标签，后期在使用 gazebo 仿真时，才需要使用到，用于配置仿真环境所需参数，比如: 机器人材料属性、gazebo插件等，但是该标签不是机器人模型必须的，只有在仿真时才需设置

**robot**
urdf 中为了保证 xml 语法的完整性，使用了robot标签作为根标签，所有的 link 和 joint 以及其他标签都必须包含在 robot 标签内,在该标签内可以通过 name 属性设置机器人模型的名称

**link**
urdf 中的 link 标签用于描述机器人某个部件(也即刚体部分)的外观和物理属性，比如: 机器人底座、轮子、激光雷达、摄像头...每一个部件都对应一个 link, 在 link 标签内，可以设计该部件的形状、尺寸、颜色、惯性矩阵、碰撞参数等一系列属性

1.属性
name ---> 为连杆命名
2.子标签
visual ---> 描述外观(对应的数据是可视的)

geometry 设置连杆的形状

标签1: box(盒状)

属性:size=长(x) 宽(y) 高(z)
标签2: cylinder(圆柱)

属性:radius=半径 length=高度
标签3: sphere(球体)

属性:radius=半径
标签4: mesh(为连杆添加皮肤)

属性: filename=资源路径(格式:package://<packagename>/<path>/文件)
origin 设置偏移量与倾斜弧度

属性1: xyz=x偏移 y便宜 z偏移

属性2: rpy=x翻滚 y俯仰 z偏航 (单位是弧度)

metrial 设置材料属性(颜色)

属性: name

标签: color

属性: rgba=红绿蓝权重值与透明度 (每个权重值以及透明度取值[0,1])
collision ---> 连杆的碰撞属性

Inertial ---> 连杆的惯性矩阵

在此，只演示visual使用。

    <link name="base_link">
        <visual>
            <!-- 形状 -->
            <geometry>
                <!-- 长方体的长宽高 -->
                <!-- <box size="0.5 0.3 0.1" /> -->
                <!-- 圆柱，半径和长度 -->
                <!-- <cylinder radius="0.5" length="0.1" /> -->
                <!-- 球体，半径-->
                <!-- <sphere radius="0.3" /> -->

            </geometry>
            <!-- xyz坐标 rpy翻滚俯仰与偏航角度(3.14=180度 1.57=90度) -->
            <origin xyz="0 0 0" rpy="0 0 0" />
            <!-- 颜色: r=red g=green b=blue a=alpha -->
            <material name="black">
                <color rgba="0.7 0.5 0 0.5" />
            </material>
        </visual>
    </link>

**joint**
urdf 中的 joint 标签用于描述机器人关节的运动学和动力学属性，还可以指定关节运动的安全极限，机器人的两个部件(分别称之为 parent link 与 child link)以"关节"的形式相连接，不同的关节有不同的运动形式: 旋转、滑动、固定、旋转速度、旋转角度限制....,比如:安装在底座上的轮子可以360度旋转，而摄像头则可能是完全固定在底座上。

joint标签对应的数据在模型中是不可见的

1.属性
name ---> 为关节命名

type ---> 关节运动形式

continuous: 旋转关节，可以绕单轴无限旋转

revolute: 旋转关节，类似于 continues,但是有旋转角度限制

prismatic: 滑动关节，沿某一轴线移动的关节，有位置极限

planer: 平面关节，允许在平面正交方向上平移或旋转

floating: 浮动关节，允许进行平移、旋转运动

fixed: 固定关节，不允许运动的特殊关节

2.子标签
parent(必需的)

parent link的名字是一个强制的属性：

link:父级连杆的名字，是这个link在机器人结构树中的名字。
child(必需的)

child link的名字是一个强制的属性：

link:子级连杆的名字，是这个link在机器人结构树中的名字。
origin

属性: xyz=各轴线上的偏移量 rpy=各轴线上的偏移弧度。
axis

属性: xyz用于设置围绕哪个关节轴运动。

<!-- 
    需求: 创建机器人模型，底盘为长方体，
         在长方体的前面添加一摄像头，
         摄像头可以沿着 Z 轴 360 度旋转

 -->
<robot name="mycar">
    <!-- 底盘 -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="blue">
                <color rgba="0 0 1.0 0.5" />
            </material>
        </visual>
    </link>

    <!-- 摄像头 -->
    <link name="camera">
        <visual>
            <geometry>
                <box size="0.02 0.05 0.05" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="red">
                <color rgba="1 0 0 0.5" />
            </material>
        </visual>
    </link>

    <!-- 关节 -->
    <joint name="camera2baselink" type="continuous">
        <parent link="base_link"/>
        <child link="camera" />
        <!-- 需要计算两个 link 的物理中心之间的偏移量 -->
        <origin xyz="0.2 0 0.075" rpy="0 0 0" />
        <axis xyz="0 0 1" />
    </joint>

</robot>

<launch>

    <param name="robot_description" textfile="$(find urdf_rviz_demo)/urdf/urdf/urdf03_joint.urdf" />
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find urdf_rviz_demo)/config/helloworld.rviz" /> 

    <!-- 添加关节状态发布节点 -->
    <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
    <!-- 添加机器人状态发布节点 -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
    <!-- 可选:用于控制关节运动的节点 -->
    <node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" />

</launch>



1.check_urdf 语法检查
进入urdf文件所属目录，调用:check_urdf urdf文件，如果不抛出异常，说明文件合法,否则非法s

2.urdf_to_graphiz 结构查看
进入urdf文件所属目录，调用:urdf_to_graphiz urdf文件，当前目录下会生成 pdf 文件

**xacro**
如果在编程语言中，可以通过变量结合函数直接解决上述问题，在 ROS 中，已经给出了类似编程的优化方案，称之为:Xacro
Xacro 是 XML Macros 的缩写，Xacro 是一种 XML 宏语言，是可编程的 XML。

rosrun xacro xacro xxx.xacro > xxx.urdf

在使用 xacro 生成 urdf 时，根标签robot中必须包含命名空间声明:xmlns:xacro="http://wiki.ros.org/xacro"


<xacro:property name="xxxx" value="yyyy" />
${属性名称}${数学表达式}
<robot name="xxx" xmlns:xacro="http://wiki.ros.org/xacro">
      <xacro:include filename="my_base.xacro" />
      <xacro:include filename="my_camera.xacro" />
      <xacro:include filename="my_laser.xacro" />
      ....
</robot>

实现分析:

机器人模型由多部件组成，可以将不同组件设置进单独文件，最终通过文件包含实现组件的拼装。

实现流程:

首先编写摄像头和雷达的 xacro 文件

然后再编写一个组合文件，组合底盘、摄像头与雷达

最后，通过 launch 文件启动 Rviz 并显示模型

**arbotix**
通过 URDF 结合 rviz 可以创建并显示机器人模型，不过，当前实现的只是静态模型，如何控制模型的运动呢？在此，可以调用 Arbotix 实现此功能。

Arbotix:Arbotix 是一款控制电机、舵机的控制板，并提供相应的 ros 功能包，这个功能包的功能不仅可以驱动真实的 Arbotix 控制板，它还提供一个差速控制器，通过接受速度控制指令更新机器人的 joint 状态，从而帮助我们实现机器人在 rviz 中的运动。

写 yaml 文件，启动 arbotix 和 rviz

<node name="arbotix" pkg="arbotix_python" type="arbotix_driver" output="screen">
     <rosparam file="$(find my_urdf05_rviz)/config/hello.yaml" command="load" />
     <param name="sim" value="true" />
</node>

此时调用 rostopic list 会发现一个熟悉的话题: /cmd_vel

**gazebo**
注意， 当 URDF 需要与 Gazebo 集成时，和 Rviz 有明显区别:

1.必须使用 collision 标签，因为既然是仿真环境，那么必然涉及到碰撞检测，collision 提供碰撞检测的依据。

2.必须使用 inertial 标签，此标签标注了当前机器人某个刚体部分的惯性矩阵，用于一些力学相关的仿真计算。

3.颜色设置，也需要重新使用 gazebo 标签标注，因为之前的颜色设置为了方便调试包含透明度，仿真环境下没有此选项。

较之于 rviz，gazebo在集成 URDF 时，需要做些许修改，比如:必须添加 collision 碰撞属性相关参数、必须添加 inertial 惯性矩阵相关参数，另外，如果直接移植 Rviz 中机器人的颜色设置是没有显示的，颜色设置也必须做相应的变更。

2.inertial
惯性矩阵的设置需要结合link的质量与外形参数动态生成，标准的球体、圆柱与立方体的惯性矩阵公式如下(已经封装为 xacro 实现):

球体惯性矩阵

需要注意的是，原则上，除了 base_footprint 外，机器人的每个刚体部分都需要设置惯性矩阵，且惯性矩阵必须经计算得出，如果随意定义刚体部分的惯性矩阵，那么可能会导致机器人在 Gazebo 中出现抖动，移动等现象

Gazebo 中创建仿真实现方式有两种:

方式1: 直接添加内置组件创建仿真环境

方式2: 手动绘制仿真环境(更为灵活)

也还可以直接下载使用官方或第三方提高的仿真环境插件。

核心代码: 启动 empty_world 后，再根据arg加载自定义的仿真环境

3.1 下载官方模型库
git clone https://github.com/osrf/gazebo_models

之前是:hg clone https://bitbucket.org/osrf/gazebo_models但是已经不可用

注意: 此过程可能比较耗时

3.2 将模型库复制进 gazebo
将得到的gazebo_models文件夹内容复制到 /usr/share/gazebo-*/models

3.3 应用
重启 Gazebo，选择左侧菜单栏的 insert 可以选择并插入相关道具了

## 综合应用
URDF 用于创建机器人模型、Rviz 可以显示机器人感知到的环境信息，Gazebo 用于仿真，可以模拟外界环境，以及机器人的一些传感器，如何在 Gazebo 中运行这些传感器

运动控制以及里程计信息显示

雷达信息仿真以及显示

摄像头信息仿真以及显示

kinect 信息仿真以及显示

ros_control 控制机器人运动
ros_control 是一套机器人控制的中间件，是一套规范，不同的机器人平台只要按照这套规范实现，那么就可以保证 与ROS 程序兼容，通过这套规范，实现了一种可插拔的架构设计，大大提高了程序设计的效率与灵活性。

已经创建完毕的机器人模型，编写一个单独的 xacro 文件，为机器人模型添加传动装置以及控制器

将此文件集成进xacro文件

启动 Gazebo 并发布 /cmd_vel 消息控制机器人运动

rviz是三维可视化工具，强调把已有的数据可视化显示；

gazebo是三维物理仿真平台，强调的是创建一个虚拟的仿真环境。

rviz需要已有数据。

rviz提供了很多插件，这些插件可以显示图像、模型、路径等信息，但是前提都是这些数据已经以话题、参数的形式发布，rviz做的事情就是订阅这些数据，并完成可视化的渲染，让开发者更容易理解数据的意义。

## 机器人导航
秉着"不重复发明轮子"的原则，ROS 中导航相关的功能包集为机器人导航提供了一套通用的实现，开发者不再需要关注于导航算法、硬件交互... 等偏复杂、偏底层的实现
![alt text](image-3.png)
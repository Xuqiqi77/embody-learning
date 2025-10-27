**通信机制进阶**

## 常用API(c++)
**初始化**
/** @brief ROS初始化函数。
 *
 * 该函数可以解析并使用节点启动时传入的参数(通过参数设置节点名称、命名空间...) 
 *
 * 该函数有多个重载版本，如果使用NodeHandle建议调用该版本。 
 *
 * \param argc 参数个数
 * \param argv 参数列表
 * \param name 节点名称，需要保证其唯一性，不允许包含命名空间
 * \param options 节点启动选项，被封装进了ros::init_options
 *
 */
void init(int &argc, char **argv, const std::string& name, uint32_t options = 0);

**发布对象**
/**
* \brief 根据话题生成发布对象
*
* 在 ROS master 注册并返回一个发布者对象，该对象可以发布消息
*
* 使用示例如下:
*
*   ros::Publisher pub = handle.advertise<std_msgs::Empty>("my_topic", 1);
*
* \param topic 发布消息使用的话题
*
* \param queue_size 等待发送给订阅者的最大消息数量
*
* \param latch (optional) 如果为 true,该话题发布的最后一条消息将被保存，并且后期当有订阅者连接时会将该消息发送给订阅者
*
* \return 调用成功时，会返回一个发布对象
*
*
*/
template <class M>
Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)

/**
* 发布消息          
*/
template <typename M>
void publish(const M& message) const

**订阅对象**
/**
   * \brief 生成某个话题的订阅对象
   *
   * 该函数将根据给定的话题在ROS master 注册，并自动连接相同主题的发布方，每接收到一条消息，都会调用回调
   * 函数，并且传入该消息的共享指针，该消息不能被修改，因为可能其他订阅对象也会使用该消息。
   * 
   * 使用示例如下:

void callback(const std_msgs::Empty::ConstPtr& message)
{
}

ros::Subscriber sub = handle.subscribe("my_topic", 1, callback);

   *
* \param M [template] M 是指消息类型
* \param topic 订阅的话题
* \param queue_size 消息队列长度，超出长度时，头部的消息将被弃用
* \param fp 当订阅到一条消息时，需要执行的回调函数
* \return 调用成功时，返回一个订阅者对象，失败时，返回空对象
* 

void callback(const std_msgs::Empty::ConstPtr& message){...}
ros::NodeHandle nodeHandle;
ros::Subscriber sub = nodeHandle.subscribe("my_topic", 1, callback);
if (sub) // Enter if subscriber is valid
{
...
}

*/
template<class M>
Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr<M const>&), const TransportHints& transport_hints = TransportHints())


**服务对象**
/**
* \brief 生成服务端对象
*
* 该函数可以连接到 ROS master，并提供一个具有给定名称的服务对象。
*
* 使用示例如下:
\verbatim
bool callback(std_srvs::Empty& request, std_srvs::Empty& response)
{
return true;
}

ros::ServiceServer service = handle.advertiseService("my_service", callback);
\endverbatim
*
* \param service 服务的主题名称
* \param srv_func 接收到请求时，需要处理请求的回调函数
* \return 请求成功时返回服务对象，否则返回空对象:
\verbatim
bool Foo::callback(std_srvs::Empty& request, std_srvs::Empty& response)
{
return true;
}
ros::NodeHandle nodeHandle;
Foo foo_object;
ros::ServiceServer service = nodeHandle.advertiseService("my_service", callback);
if (service) // Enter if advertised service is valid
{
...
}
\endverbatim

*/
template<class MReq, class MRes>
ServiceServer advertiseService(const std::string& service, bool(*srv_func)(MReq&, MRes&))

**客户端对象**
/** 
  * @brief 创建一个服务客户端对象
  *
  * 当清除最后一个连接的引用句柄时，连接将被关闭。
  *
  * @param service_name 服务主题名称
  */
 template<class Service>
 ServiceClient serviceClient(const std::string& service_name, bool persistent = false, 
                             const M_string& header_values = M_string())

/**
   * @brief 发送请求
   * 返回值为 bool 类型，true，请求处理成功，false，处理失败。
   */
  template<class Service>
  bool call(Service& service)

/**
 * ros::service::waitForService("addInts");
 * \brief 等待服务可用，否则一致处于阻塞状态
 * \param service_name 被"等待"的服务的话题名称
 * \param timeout 等待最大时常，默认为 -1，可以永久等待直至节点关闭
 * \return 成功返回 true，否则返回 false。
 */
ROSCPP_DECL bool waitForService(const std::string& service_name, ros::Duration timeout = ros::Duration(-1));

/**
* client.waitForExistence();
* \brief 等待服务可用，否则一致处于阻塞状态
* \param timeout 等待最大时常，默认为 -1，可以永久等待直至节点关闭
* \return 成功返回 true，否则返回 false。
*/
bool waitForExistence(ros::Duration timeout = ros::Duration(-1));


**回旋函数**

/**
 * \brief 处理一轮回调
 *
 * 一般应用场景:
 *     在循环体内，处理所有可用的回调函数
 * 
 */
ROSCPP_DECL void spinOnce();


/** 
 * \brief 进入循环处理回调 
 */
ROSCPP_DECL void spin();

ros::spin() 是进入了循环执行回调函数，而 ros::spinOnce() 只会执行一次回调函数(没有循环)，在 ros::spin() 后的语句不会执行到，而 ros::spinOnce() 后的语句可以执行。

**时间**  

时刻

ros::init(argc,argv,"hello_time");
ros::NodeHandle nh;//必须创建句柄，否则时间没有初始化，导致后续API调用失败
ros::Time right_now = ros::Time::now();//将当前时刻封装成对象
ROS_INFO("当前时刻:%.2f",right_now.toSec());//获取距离 1970年01月01日 00:00:00 的秒数
ROS_INFO("当前时刻:%d",right_now.sec);//获取距离 1970年01月01日 00:00:00 的秒数

ros::Time someTime(100,100000000);// 参数1:秒数  参数2:纳秒
ROS_INFO("时刻:%.2f",someTime.toSec()); //100.10
ros::Time someTime2(100.3);//直接传入 double 类型的秒数
ROS_INFO("时刻:%.2f",someTime2.toSec()); //100.30

**持续时间**
ROS_INFO("当前时刻:%.2f",ros::Time::now().toSec());
ros::Duration du(10);//持续10秒钟,参数是double类型的，以秒为单位
du.sleep();//按照指定的持续时间休眠
ROS_INFO("持续时间:%.2f",du.toSec());//将持续时间换算成秒
ROS_INFO("当前时刻:%.2f",ros::Time::now().toSec());

**持续时间与时刻运算**
ROS_INFO("时间运算");
ros::Time now = ros::Time::now();
ros::Duration du1(10);
ros::Duration du2(20);
ROS_INFO("当前时刻:%.2f",now.toSec());
//1.time 与 duration 运算
ros::Time after_now = now + du1;
ros::Time before_now = now - du1;
ROS_INFO("当前时刻之后:%.2f",after_now.toSec());
ROS_INFO("当前时刻之前:%.2f",before_now.toSec());

//2.duration 之间相互运算
ros::Duration du3 = du1 + du2;
ros::Duration du4 = du1 - du2;
ROS_INFO("du3 = %.2f",du3.toSec());
ROS_INFO("du4 = %.2f",du4.toSec());
//PS: time 与 time 不可以运算
// ros::Time nn = now + before_now;//异常

**设置运行频率**
ros::Rate rate(1);//指定频率
while (true)
{
    ROS_INFO("-----------code----------");
    rate.sleep();//休眠，休眠时间 = 1 / 频率。
}

**定时器**
ros::NodeHandle nh;//必须创建句柄，否则时间没有初始化，导致后续API调用失败

 // ROS 定时器
 /**
* \brief 创建一个定时器，按照指定频率调用回调函数。
*
* \param period 时间间隔
* \param callback 回调函数
* \param oneshot 如果设置为 true,只执行一次回调函数，设置为 false,就循环执行。
* \param autostart 如果为true，返回已经启动的定时器,设置为 false，需要手动启动。
*/
 //Timer createTimer(Duration period, const TimerCallback& callback, bool oneshot = false,
 //                bool autostart = true) const;

 // ros::Timer timer = nh.createTimer(ros::Duration(0.5),doSomeThing);
 ros::Timer timer = nh.createTimer(ros::Duration(0.5),doSomeThing,true);//只执行一次

 // ros::Timer timer = nh.createTimer(ros::Duration(0.5),doSomeThing,false,false);//需要手动启动
 // timer.start();
 ros::spin(); //必须 spin

void doSomeThing(const ros::TimerEvent &event){
    ROS_INFO("-------------");
    ROS_INFO("event:%s",std::to_string(event.current_real.toSec()).c_str());
}

**其他函数**
1.节点状态判断

/** \brief 检查节点是否已经退出
 *
 *  ros::shutdown() 被调用且执行完毕后，该函数将会返回 false
 *
 * \return true 如果节点还健在, false 如果节点已经火化了。
 */
bool ok();
Copy
2.节点关闭函数

/*
*   关闭节点
*/
void shutdown();
Copy
3.日志函数

使用示例

ROS_DEBUG("hello,DEBUG"); //不会输出
ROS_INFO("hello,INFO"); //默认白色字体
ROS_WARN("Hello,WARN"); //默认黄色字体
ROS_ERROR("hello,ERROR");//默认红色字体
ROS_FATAL("hello,FATAL");//默认红色字体

## ROS中的头文件与源文件
设置头文件，可执行文件作为源文件；
分别设置头文件，源文件与可执行文件。

**自定义头** 
 "/home/用户/工作空间/src/功能包/include/**"
可执行文件
在 src 目录下新建文件:hello.cpp
#include "test_head/hello.h"
namespace hello_ns {

void HelloPub::run(){
    ROS_INFO("自定义头文件的使用....");
}

}

配置文件
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hello src/hello.cpp)

add_dependencies(hello ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hello
  ${catkin_LIBRARIES}
)

**自定义源文件**
编写头文件；
编写源文件；
编写可执行文件；
编辑配置文件并执行



## ROS运行管理
ROS是多进程(节点)的分布式框架，一个完整的ROS系统实现：

可能包含多台主机；
每台主机上又有多个工作空间(workspace)；
每个的工作空间中又包含多个功能包(package)；
每个功能包又包含多个节点(Node)，不同的节点都有自己的节点名称；
每个节点可能还会设置一个或多个话题(topic)...

**ROS元功能包**

在ROS中，提供了一种方式可以将不同的功能包打包成一个功能包，当安装某个功能模块时，直接调用打包后的功能包即可，该包又称之为元功能包(metapackage)。

MetaPackage是Linux的一个文件管理系统的概念。是ROS中的一个虚包，里面没有实质性的内容，但是它依赖了其他的软件包，通过这种方法可以把其他包组合起来，我们可以认为它是一本书的目录索引，告诉我们这个包集合中有哪些子包，并且该去哪里下载。

**ROS节点运行管理launch文件**

概念
launch 文件是一个 XML 格式的文件，可以启动本地和远程的多个节点，还可以在参数服务器中设置参数。

作用
简化节点的配置与启动，提高ROS程序的启动效率。

roslaunch 命令执行launch文件时，首先会判断是否启动了 roscore,如果启动了，则不再启动，否则，会自动调用 roscore

**launch文件标签**

<launch>标签是所有 launch 文件的根标签，充当其他标签的容器
<node>标签用于指定 ROS 节点，是最常见的标签，需要注意的是: roslaunch 命令不能保证按照 node 的声明顺序来启动节点(节点的启动是多进程的)
{
    pkg="包名"

节点所属的包

type="nodeType"

节点类型(与之相同名称的可执行文件)

name="nodeName"

节点名称(在 ROS 网络拓扑中节点的名称)

args="xxx xxx xxx" (可选)

将参数传递给节点

machine="机器名"

在指定机器上启动节点

respawn="true | false" (可选)

如果节点退出，是否自动重启

respawn_delay=" N" (可选)

如果 respawn 为 true, 那么延迟 N 秒后启动节点

required="true | false" (可选)

该节点是否必须，如果为 true,那么如果该节点退出，将杀死整个 roslaunch

ns="xxx" (可选)

在指定命名空间 xxx 中启动节点

clear_params="true | false" (可选)

在启动前，删除节点的私有空间的所有参数

output="log | screen" (可选)

日志发送目标，可以设置为 log 日志文件，或 screen 屏幕,默认是 log
}
{
    env 环境变量设置

remap 重映射节点名称

rosparam 参数设置

param 参数设置
}

<include>标签用于将另一个 xml 格式的 launch 文件导入到当前文件
<remap>用于话题重命名
<param>标签主要用于在参数服务器上设置参数，参数源可以在标签中通过 value 指定，也可以通过外部文件加载，在<node>标签中时，相当于私有命名空间。
<rosparam>标签可以从 YAML 文件导入参数，或将参数导出到 YAML 文件，也可以用来删除参数，<rosparam>标签在<node>标签中时被视为私有。
<group>标签可以对节点分组，具有 ns 属性，可以让节点归属某个命名空间
<arg>标签是用于动态传参，类似于函数的参数，可以增强launch文件的灵活性

**ROS工作空间覆盖**
所谓工作空间覆盖，是指不同工作空间中，存在重名的功能包的情形。
0.新建工作空间A与工作空间B，两个工作空间中都创建功能包: turtlesim。
1.在 ~/.bashrc 文件下追加当前工作空间的 bash 格式如下:
source /home/用户/路径/工作空间A/devel/setup.bash
source /home/用户/路径/工作空间B/devel/setup.bash
2.新开命令行:source .bashrc加载环境变量

3.查看ROS环境环境变量echo $ROS_PACKAGE_PATH

结果:自定义工作空间B:自定义空间A:系统内置空间

4.调用命令:roscd turtlesim会进入自定义工作空间B

结论
功能包重名时，会按照 ROS_PACKAGE_PATH 查找，配置在前的会优先执行。

结论
功能包重名时，会按照 ROS_PACKAGE_PATH 查找，配置在前的会优先执行。

隐患
存在安全隐患，比如当前工作空间B优先级更高，意味着当程序调用 turtlesim 时，不会调用工作空间A也不会调用系统内置的 turtlesim，如果工作空间A在实现时有其他功能包依赖于自身的 turtlesim，而按照ROS工作空间覆盖的涉及原则，那么实际执行时将会调用工作空间B的turtlesim，从而导致执行异常，出现安全隐患。

**ROS节点名称重名**

在ROS中给出的解决策略是使用命名空间或名称重映射。

命名空间就是为名称添加前缀，名称重映射是为名称起别名。这两种策略都可以解决节点重名问题，两种策略的实现途径有多种:

rosrun 命令
launch 文件
编码实现


rosrun 设置命名空间:
rosrun 包名 节点名 __ns:=新名称
rosrun turtlesim turtlesim_node __ns:=/xxx

rosnode list查看节点信息,显示结果:
/xxx/turtlesim
/yyy/turtlesim

rosrun名称重映射
rosrun 包名 节点名 __name:=新名称
rosrun turtlesim  turtlesim_node __name:=t1
rosrun turtlesim  turtlesim_node __name:=t2

rosnode list查看节点信息,显示结果:
/t1
/t2

设置命名空间同时名称重映射
语法: rosrun 包名 节点名 __ns:=新名称 __name:=新名称


launch设置
<launch>

    <node pkg="turtlesim" type="turtlesim_node" name="t1" />
    <node pkg="turtlesim" type="turtlesim_node" name="t2" />
    <node pkg="turtlesim" type="turtlesim_node" name="t1" ns="hello"/>

</launch>

/t1
/t2
/t1/hello


编码设置


----------
全局(参数名称直接参考ROS系统，与节点命名空间平级)
相对(参数名称参考的是节点的命名空间，与节点名称平级)
私有(参数名称参考节点名称，是节点名称的子级)
--------
**ROS话题名称设置**

在ROS中节点名称可能出现重名的情况，同理话题名称也可能重名。

rosrun 命令
launch 文件
编码实现

rosrun 
rosrun turtlesim turtlesim_node /turtle1/cmd_vel:=/cmd_vel

launch
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="t1">
        <remap from="/turtle1/cmd_vel" to="/cmd_vel" />
    </node>
    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="key" />

</launch>

**参数重名**

关于参数重名的处理，没有重映射实现，为了尽量的避免参数重名，都是使用为参数名添加前缀的方式，实现类似于话题名称，有全局、相对、和私有三种类型之分。

rosrun turtlesim turtlesim_node _A:=100 设置参数A的值为100
launch
<launch>

    <param name="p1" value="100" />
    <node pkg="turtlesim" type="turtlesim_node" name="t1">
        <param name="p2" value="100" />
    </node>

</launch>

**ROS分布式通信**

所有端口上的所有机器之间必须有完整的双向连接。
每台计算机必须通过所有其他计算机都可以解析的名称来公告自己。

先要保证不同计算机处于同一网络中，最好分别设置固定IP，如果为虚拟机，需要将网络适配器改为桥接模式；

分别修改不同计算机的 /etc/hosts 文件，在该文件中加入对方的IP地址和计算机名:
主机
export ROS_MASTER_URI=http://主机IP:11311
export ROS_HOSTNAME=主机IP
从机
export ROS_MASTER_URI=http://主机IP:11311
export ROS_HOSTNAME=从机IP

## ROS常用组件
TF坐标变换，实现不同类型的坐标系之间的转换；
rosbag 用于录制ROS节点的执行过程并可以重放该过程；
rqt 工具箱，集成了多款图形化的调试工具。

**TF坐标变换**

transform frame
msg:geometry_msgs/TransformStamped和geometry_msgs/PointStamped

所谓静态坐标变换，是指两个坐标系之间的相对位置是固定的。

所谓动态坐标变换，是指两个坐标系之间的相对位置是变化的。

rosrun rviz rviz

多坐标变换:
现有坐标系统，父级坐标系统 world,下有两子级系统 son1，son2，son1 相对于 world，以及 son2 相对于 world 的关系是已知的，求 son1原点在 son2中的坐标，又已知在 son1中一点的坐标，要求求出该点在 son2 中的坐标

多坐标变换其实就是：
让系统能自动推导出任意两个坐标系之间的关系。
你只要告诉它每个节点和父节点的关系，TF 会自动拼出整条变换链

启动坐标系广播程序之后，运行如下命令:
rosrun tf2_tools view_frames.py

会产生类似于下面的日志信息:
[INFO] [1592920556.827549]: Listening to tf data during 5 seconds...
[INFO] [1592920561.841536]: Generating graph in frames.pdf file...

**Rosbag**
在ROS中关于数据的留存以及读取实现，提供了专门的工具: rosbag。
rosbag本质也是ros的节点，当录制时，rosbag是一个订阅节点，可以订阅话题消息并将订阅到的数据写入磁盘文件；当重放时，rosbag是一个发布节点，可以读取磁盘文件，发布文件中的话题消息。

**RQT工具箱**
rqt 工具箱组成有三大部分

rqt——核心实现，开发人员无需关注

rqt_common_plugins——rqt 中常用的工具套件

rqt_robot_plugins——运行中和机器人交互的插件(比如: rviz)


rqt_graph: 显示当前系统中所有节点以及节点之间的连接关系
rqt_console 是 ROS 中用于显示和过滤日志的图形化插件
rqt_plot 图形绘制插件，可以以 2D 绘图的方式绘制发布在 topic 上的数据
rqt_bag 简介:录制和重放 bag 文件的图形化插件
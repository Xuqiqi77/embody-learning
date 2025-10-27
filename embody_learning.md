**1. INTRO**

manipulation : 精细行为
lowlevel - task level

ROS: Robot Operating System

camera = Perception  - = planning = rbt control = motor control
         robot state -

差分语言:
- print("Hello world")
+ print("Hello embody learning")
这段 diff 本身就是一种“差分语言”，Git 就在用它。

ros使用差分方程语言

drake  - 机器人动力学 库

**2. robot**

关节位置sensor 关节扭矩sensor touch sensor
most:positon-control

elcectric motor: 希望高扭矩 低速度 pidcontrol

torque control: safety

重力补偿

**3.1 basic pick and place**

context: x[n+1]= f(x[n],u[n],w[n],n,p)
                state, input, noise, time, parameter

struct context{
    x,u,w,n,p
}

空间代数-抓取器轨迹-机器人轨迹


point A  P(A) = (x,y,z) position of point A
         P(A,B) = (x,y,z) position of point A to point B
         P(A,B)[C] position of point A to point B 在C为原点坐标系下的位置
右手坐标系 XYZ-RGB

rotation : R(B,A)   frame B to frame A 

rotation matrix: 3*3
Axis angle: 旋转轴和旋转角度
unit quaternion: 
euler angle: 

addition: 类似向量    P(A,B) + P(B,C) = P(A,C)   p(A,B) = -P(B,A)
multiplication:   p(A,B)[G] = R(G,F) * P(A,B)[F]
mutliplication inverse:  R(A,B) * R(B,A) = R(A,A) = I
                         R(A,B) * R(B,C) = R(A,C)
                         R(A,B)-1 = R(B,A)
                
X(A,B) special pose of B relative to A   means {P(A,B)[A],R(A,B)}
pose transform:

Given: 
X(W,Oinital) X(W,Ginitial) X(W,Ogoal)  G:grab O:object  W:world

R(θ) = [cos(θ) ,-sin(θ); sin(θ), cos(θ)] Ex 2*2 rotation matrix

slurp 

关节:可以分别用body1和body2来表示
add body: +6 freedom
add joint: 根据关节类型减少自由度

**3.2 basic pick and place**

rotation matrix: 3*3 Rt * R = I
欧拉角 Roll Pitch Yaw no constraints
Axis angle: 旋转轴和旋转角度 [x;y;z] no constraints
unit quaternion: w,x,y,z   1 constraints w^2 + x^2 + y^2 + z^2 = 1

V(A,B)[G] = R[G,F]*V(A,B)[F]
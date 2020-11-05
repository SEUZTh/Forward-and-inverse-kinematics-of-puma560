<center><font size=6><b>机器人技术基础实验2报告</b></font></center>
<center>张天浩</center>
<center>2020.10.19</center>

<!-- TOC -->

- [1. 实验目的和要求](#1-实验目的和要求)
- [2. 实验手段](#2-实验手段)
    - [2.1 实验原理](#21-实验原理)
    - [2.2 函数详解](#22-函数详解)
- [3. 实验主要内容](#3-实验主要内容)
- [4. 实验结果和分析](#4-实验结果和分析)
    - [4.1 正运动学求解](#41-正运动学求解)
        - [例1](#例1)
        - [例2](#例2)
    - [4.2 逆运动学求解](#42-逆运动学求解)
        - [例1](#例1-1)
        - [例2](#例2-1)

<!-- /TOC -->

# 1. 实验目的和要求
- 掌握常见形式的机器人的D-H坐标系建立方法、运动学建模；
- 正逆运动学分析与求解；
- 使用Robotics Toolbox对直角坐标机器人、圆柱坐标机器人、球坐标机器人、PUMA560其中之一建立运动学模型，正逆求解和仿真。
# 2. 实验手段
## 2.1 实验原理
- D-H坐标系
- 正运动学求解：已知机器人的所有连杆长度和关节角度，计算机器人手的位姿。
- 逆运动学求解：找到方程的逆，求得所需的关节变量，使机器人放置在期望的位姿。
- Robotics Toolbox & MATLAB
## 2.2 函数详解
- transl
    - Create or unpack an SE3 translational transform
    - ```T = transl(x, y, z)``` is an SE(3) homogeneous transform (4x4) representing a pure translation of x, y and z.
```matlab
>> T=transl(0, 0, 0.62)

T =

    1.0000         0         0         0
         0    1.0000         0         0
         0         0    1.0000    0.6200
         0         0         0    1.0000
```
---
- trotz
    - Rotation about Z axis
    - ```T = trotz(theta)``` is a homogeneous transformation (4x4) representing a rotation of theta radians about the z-axis.
```matlab
>> T=trotz(pi/2)

T =

    0.9996   -0.0274         0         0
    0.0274    0.9996         0         0
         0         0    1.0000         0
         0         0         0    1.0000
```
---
- SerialLink
    - Serial-link robot class
    - A concrete class that represents a serial-link arm-type robot. The mechanism is described using Denavit-Hartenberg parameters, one set per joint.
```matlab
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','puma560','base' , ...
transl(1, 1, 1)* trotz(pi/2)); %连接连杆，机器人取名puma560
```
---
- Link
    - Robot manipulator Link class
    - A Link object holds all information related to a robot link such as kinematics parameters, rigid-body inertial parameters, motor and transmission parameters.
```matlab
%             theta      d           a          alpha       offset
L1=Link([   0          0            0           -pi/2         0 ],'standard'); %连杆的D-H参数
L2=Link([   0          0          0.4318        0             0 ],'standard');
L3=Link([   0       0.1501	      0.0203       pi/2           0 ],'standard');
L4=Link([   0       0.4318          0          -pi/2          0 ],'standard');
L5=Link([   0          0            0          pi/2           0 ],'standard');
L6=Link([   0          0            0           0             0 ],'standard');
```
---
# 3. 实验主要内容
- PUMA机器人的标准D-H参数如下：

|连杆|$\theta_{i}$|$d$|$a$|$\alpha$|
|:---:|:---:|:---:|:---:|:---:|
|1|$\theta_{1}$|0|0|-90|
|2|$\theta_{2}$|149.09mm|431.8mm|0|
|3|$\theta_{3}$|0|-20.32mm|90|
|4|$\theta_{4}$|433.07mm|0|-90|
|5|$\theta_{5}$|0|0|90|
|6|$\theta_{6}$|56.25mm|0|0|
# 4. 实验结果和分析
## 4.1 正运动学求解
### 例1
$\theta=[\frac{\pi}{2},0,\frac{\pi}{2},0,0,0]$
```matlab
>> theta=[pi/2 0 pi/2 0 0 0];
>> T=robot.fkine(theta)
 

T = 
         0        -1         0   -0.1501
         0         0         1    0.8636
        -1         0         0    0.4797
         0         0         0         1
```
![正运动学求解1](https://gitee.com/zhang_ma_nong/image-bed/raw/master/img/20201017211026.png)


### 例2
$\theta=[-\frac{\pi}{2},0,-\frac{\pi}{2},0,0,0]$
```matlab
>> theta=[-pi/2 0 -pi/2 0 0 0];
>> T=robot.fkine(theta)
 

T = 
         0         1         0    0.1501
         0         0         1         0
         1         0         0    0.5203
         0         0         0         1
```
![正运动学求解2](https://gitee.com/zhang_ma_nong/image-bed/raw/master/img/20201017212415.png)

## 4.2 逆运动学求解
### 例1
```matlab
bx = -0.06;
by = -0.2;
bz = 0.1;
tform = rpy2tr(136,-180,-180); %欧拉角转姿态齐次矩阵
targetPos = [bx by bz]; %末端位置向量
TR=transl(targetPos)*tform; %位姿齐次矩阵
q=robot.ikine(TR); % 求逆解
>> q

q =

   -2.6644    0.1658   -2.6476   -0.7275    1.9528    2.1979
```
![逆运动学求解1](https://gitee.com/zhang_ma_nong/image-bed/raw/master/img/20201017214307.png)

### 例2
```matlab
```matlab
bx = -0.35;
by = -0.4;
bz = 0.2;
tform = rpy2tr(136,-180,-180); %欧拉角转姿态齐次矩阵
targetPos = [bx by bz]; %末端位置向量
TR=transl(targetPos)*tform; %位姿齐次矩阵
q=robot.ikine(TR); % 求逆解
>> q

q =

    1.1383    1.7925   -3.1260    0.3310    2.0308   -1.0988
```
![逆运动学求解2](https://gitee.com/zhang_ma_nong/image-bed/raw/master/img/20201017214154.png)

# 5. 附 Matlab 代码和演示视频
- 正逆运动学求解代码：
```matlab
%% 标准DH建立puma560机器人模型
clear;
clc;
%建立机器人模型(标准D-H参数)
theta=[-pi/2 0 -pi/2 0 0 0];
%theta=[0 0 -pi/2 pi/2 pi/2 0];
%               theta             d           a          alpha       offset
L1=Link([   theta(1)          0           0         -pi/2            0 ],'standard'); %连杆的D-H参数
L2=Link([   theta(2)          0        0.4318       0               0 ],'standard');
L3=Link([   theta(3)      0.1501	 0.0203     pi/2            0 ],'standard');
L4=Link([   theta(4)       0.4318      0          -pi/2            0 ],'standard');
L5=Link([   theta(5)          0           0            pi/2            0 ],'standard');
L6=Link([   theta(6)          0           0            0               0 ],'standard');
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','puma560','base' , ...
transl(0, 0, 0.5)* trotz(0)); %连接连杆，机器人取名puma560
% base	pose of robot's base (4x4 homog xform)
figure(1)
robot.teach() %teach可视化模型并可以单轴驱动，可以查看模型与实际机器人的关节运动是否一致。
robot.display();  %Link 类函数，显示建立的机器人DH参数

T=robot.fkine(theta);		%计算机器人正运动学，括号内为theta值
disp(T);
robot.plot(theta)

%% 使用Peter工具箱逆解
hold on
title("工具箱机器人模型");
xlabel('x/米','FontSize',12);
ylabel('y/米','FontSize',12);
zlabel('z/米','FontSize',12);
bx = -0.35;
by = -0.4;
bz = 0.2;
tform = rpy2tr(136,-180,-180); %欧拉角转姿态齐次矩阵
targetPos = [bx by bz]; %末端位置向量
TR=transl(targetPos)*tform; %位姿齐次矩阵
hold on
grid on
plot3(bx,by,bz,'*','LineWidth',1);
q=robot.ikine(TR);
robot.plot(q);
```

- 逆运动学求解——画圆
```matlab
%% 标准DH建立puma560机器人模型
clear;
clc;
%建立机器人模型(标准D-H参数)
theta=[-pi/2 0 -pi/2 0 0 0];
%theta=[0 0 -pi/2 pi/2 pi/2 0];
%               theta             d           a          alpha       offset
L1=Link([   theta(1)          0           0         -pi/2            0 ],'standard'); %连杆的D-H参数
L2=Link([   theta(2)          0        0.4318       0               0 ],'standard');
L3=Link([   theta(3)      0.1501	 0.0203     pi/2            0 ],'standard');
L4=Link([   theta(4)       0.4318      0          -pi/2            0 ],'standard');
L5=Link([   theta(5)          0           0            pi/2            0 ],'standard');
L6=Link([   theta(6)          0           0            0               0 ],'standard');
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','puma560','base' , ...
transl(0, 0, 0.5)* trotz(0)); %连接连杆，机器人取名puma560
% base	pose of robot's base (4x4 homog xform)
figure(1)
robot.teach() %teach可视化模型并可以单轴驱动，可以查看模型与实际机器人的关节运动是否一致。
robot.display();  %Link 类函数，显示建立的机器人DH参数

T=robot.fkine(theta);		%计算机器人正运动学，括号内为theta值
disp(T);
robot.plot(theta)

%% 定义圆路径
t = (0:0.2:20)'; count = length(t);center = [-0.25 0.6 0.1];radius = 0.1;%圆心和半径
theta = t*(2*pi/t(end));
points =(center + radius*[cos(theta) sin(theta) zeros(size(theta))])';
figure(1);
plot3(points(1,:),points(2,:),points(3,:),'r')

%% 使用Peter工具箱逆解进行轨迹重现
hold on
% axis([-1 1.2 -1 1.2 -1 1.2])
title("工具箱机器人模型末端运动轨迹");
xlabel('x/米','FontSize',12);
ylabel('y/米','FontSize',12);
zlabel('z/米','FontSize',12);
for i = 1:size(points,2)
    pause(0.01)
    bx = points(1,i);
    by = points(2,i);
    bz =points(3,i);
    tform = rpy2tr(136,-180,-180); %欧拉角转姿态齐次矩阵
    targetPos = [bx by bz]; %末端位置向量
    % targetOr = [0.02 -0.14 0.99 -0.01] %四元数
    % tform = quat2tform(targetOr) %四元数转姿态齐次矩阵
    TR=transl(targetPos)*tform; %位姿齐次矩阵
    hold on
    grid on
    plot3(bx,by,bz,'*','LineWidth',1);
    q=robot.ikine(TR);
    pause(0.01)
    robot.plot(q);%动画演示
end
```


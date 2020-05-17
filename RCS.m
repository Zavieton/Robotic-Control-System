%% 机器人控制原理课程设计
% Made by Zavieton
% 2020/5
% Robotics Toolbox for Matlab (release 9.10)
clear;
clc;
startup_rvc;

%% 机械臂为六自由度机械臂
clear L;

% 由于标准的PUMA560为in单位，设置单位转化因子k
k = 0.0254;

% DH法建立模型,关节角，连杆偏移，连杆长度，连杆扭转角，关节类型（0转动，1移动）
L(1) = Link([0  0        0         pi/2]);
L(2) = Link([0  0        17*k         0]);
L(3) = Link([0  6.68*k   0.5*k        -pi/2]);
L(4) = Link([0  17*k     0         pi/2]);
L(5) = Link([0  0        0        -pi/2]);
L(6) = Link([0  0        0            0]);

L(1).qlim=[-pi/2,pi/2];
L(2).qlim=[-pi,pi];
L(3).qlim=[-pi,pi];
L(4).qlim=[-pi/2,pi/2];
L(5).qlim=[-pi/2,pi/2];
L(6).qlim=[-pi/2,pi/2];

%关节角度限制
limitmax_1 = 180;
limitmin_1 = -180;
limitmax_2 = 180;
limitmin_2 = -180;
limitmax_3 = 180;
limitmin_3 = -180;

% 连接六个R型连杆成为一个机械臂
six_link=SerialLink(L,'name','six link');
% 镜像（复制）一个机械臂，起名为puma
puma = SerialLink(six_link,'name','puma560');

n = puma.n;% 关节数
puma.base = transl(0,0,24.55*k); % 设置基座高度
six_link.base = transl(0,0,24.55*k); % 设置基座高度
%显示机械臂
figure(1);
six_link.plot([0 0 0 0 0 0]);
six_link.teach(); %可以自由拖动的关节角度
hold on;

%% 画出工作空间的散点图
N=60000;    %随机次数
zero_N = zeros(N,1);
theta1=(limitmin_1+(limitmax_1-limitmin_1)*rand(N,1))*pi/180; %关节1限制
theta2=(limitmin_2+(limitmax_2-limitmin_2)*rand(N,1))*pi/180; %关节2限制
theta3=(limitmin_3+(limitmax_3-limitmin_3)*rand(N,1))*pi/180; %关节3限制

qq=[theta1,theta2,theta3,zero_N,zero_N,zero_N];

Mricx=puma.fkine(qq); %对每一个位置进行运动学解算tool位置

x=reshape(Mricx(1,4,:),N,1);
y=reshape(Mricx(2,4,:),N,1);
z=reshape(Mricx(3,4,:),N,1);

%动态过程演示
% for i=1:1:N
%     puma.plot([x(i),y(i),z(i),zero_N(i),zero_N(i),zero_N(i)]);
%     plot3(x(i),y(i),z(i),'b.','MarkerSize',0.5);%画出落点
%     hold on;
% end

plot3(x,y,z,'b.','MarkerSize',0.5);%画出落点


%% 逆运动学求解关节角
qz = [0 0 0 0 0 0];
T1 = puma.fkine(qz);
X0 = T1(1,4);Y0 = T1(2,4);Z0 = T1(3,4);

% 设置齐次位姿矩阵
X = 0.5; Y = 0.5; Z = 0.8;
T2 = [1,0,0,X;
    0,1,0,Y;
    0,0,1,Z;
    0,0,0,1];
joint_angel = puma.ikine6s(T2,'ru');

%% 圆弧轨迹规划

fprintf("圆弧轨迹规划 \n");
% 设置时间序列
endtime = 2;
t = (0 :0.05 :endtime)';
Nn = (endtime / 0.05) + 1;
% 生成轨迹规划
[theta,thetad,thetadd] = jtraj(qz,joint_angel,t);  % 改进的五次多项式插值
figure(2);
for i= 1:1:Nn
    puma.plot(theta(i,:));%动画显示
    Tra=puma.fkine(theta(i,:));
    plot3(Tra(1,4),Tra(2,4),Tra(3,4),'b.','MarkerSize',5);%画出落点
    title("圆弧轨迹规划");
    hold on;
end

Tra = puma.fkine(theta);
xx=reshape(Tra(1,4,:),Nn,1);
yy=reshape(Tra(2,4,:),Nn,1);
zz=reshape(Tra(3,4,:),Nn,1);
plot3(xx,yy,zz,'LineWidth',4);
grid on;


figure(3);
subplot(311);
plot(t,theta(:,1),'LineWidth',2);hold on;
plot(t,thetad(:,1),'LineWidth',2);hold on;
plot(t,thetadd(:,1),'LineWidth',2);
legend('角度','速度','加速度');title('关节1');
grid on;

subplot(312);
plot(t,theta(:,2),'LineWidth',2);hold on;
plot(t,thetad(:,2),'LineWidth',2);hold on;
plot(t,thetadd(:,2),'LineWidth',2);
legend('角度','速度','加速度');title('关节2');
grid on;

subplot(313);
plot(t,theta(:,3),'LineWidth',2);hold on;
plot(t,thetad(:,3),'LineWidth',2);hold on;
plot(t,thetadd(:,3),'LineWidth',2);
legend('角度','速度','加速度');title('关节3');
grid on;

figure(4);
subplot(311);
plot(t,theta(:,4),'LineWidth',2);hold on;
plot(t,thetad(:,4),'LineWidth',2);hold on;
plot(t,thetadd(:,4),'LineWidth',2);
legend('角度','速度','加速度');title('关节4');
grid on;
subplot(312);
plot(t,theta(:,5),'LineWidth',2);hold on;
plot(t,thetad(:,5),'LineWidth',2);hold on;
plot(t,thetadd(:,5),'LineWidth',2);
legend('角度','速度','加速度');title('关节5');
grid on;
subplot(313);
plot(t,theta(:,6),'LineWidth',2);hold on;
plot(t,thetad(:,6),'LineWidth',2);hold on;
plot(t,thetadd(:,6),'LineWidth',2);
legend('角度','速度','加速度');title('关节6');
grid on;


%% 直线轨迹规划
% 直线等间隔插值法
fprintf("直线轨迹规划 \n");
l = X-X0; m = Y-Y0; n = Z-Z0;
fprintf("\n空间直线方程为：  x - %f / %f =  y- %f / %f =  z - %f / %f\n",X0,l,Y0,m,Z0,m);

xco = zeros(Nn,1); yco = zeros(Nn,1); zco = zeros(Nn,1);
theta_LineTraj = zeros(Nn,6);
for i=1:1:Nn
    xco(i) = X0 + i*(X-X0)/Nn;
    yco(i) = Y0 + i*(Y-Y0)/Nn;
    zco(i) = Z0 + i*(Z-Z0)/Nn;
    T3 = [Tra(1,1,i),Tra(1,2,i),Tra(1,3,i),xco(i);
        Tra(2,1,i),Tra(2,2,i),Tra(2,3,i),yco(i);
        Tra(3,1,i),Tra(3,2,i),Tra(3,3,i),zco(i);
        0,0,0,1];
    theta_LineTraj(i,:) = puma.ikine6s(T3,'ru');
end
theta_LineTrajd = zeros(Nn,6);
theta_LineTrajdd = zeros(Nn,6);
for i = 1:1:6
    theta_LineTrajd(i,:) =  gradient(theta_LineTraj(i,:),t);
    theta_LineTrajdd(i,:) =  gradient(theta_LineTrajd(i,:),t);
end
figure(5);
for i= 1:1:Nn
    puma.plot(theta_LineTraj(i,:));%动画显示
    LineTraj=puma.fkine(theta_LineTraj(i,:));
    plot3(LineTraj(1,4),LineTraj(2,4),LineTraj(3,4),'b.','MarkerSize',5);%画出落点
    hold on;
    title("直线路径规划");
end


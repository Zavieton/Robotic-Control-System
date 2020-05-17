%% �����˿���ԭ��γ����
% Made by Zavieton
% 2020/5
% Robotics Toolbox for Matlab (release 9.10)
clear;
clc;
startup_rvc;

%% ��е��Ϊ�����ɶȻ�е��
clear L;

% ���ڱ�׼��PUMA560Ϊin��λ�����õ�λת������k
k = 0.0254;

% DH������ģ��,�ؽڽǣ�����ƫ�ƣ����˳��ȣ�����Ťת�ǣ��ؽ����ͣ�0ת����1�ƶ���
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

%�ؽڽǶ�����
limitmax_1 = 180;
limitmin_1 = -180;
limitmax_2 = 180;
limitmin_2 = -180;
limitmax_3 = 180;
limitmin_3 = -180;

% ��������R�����˳�Ϊһ����е��
six_link=SerialLink(L,'name','six link');
% ���񣨸��ƣ�һ����е�ۣ�����Ϊpuma
puma = SerialLink(six_link,'name','puma560');

n = puma.n;% �ؽ���
puma.base = transl(0,0,24.55*k); % ���û����߶�
six_link.base = transl(0,0,24.55*k); % ���û����߶�
%��ʾ��е��
figure(1);
six_link.plot([0 0 0 0 0 0]);
six_link.teach(); %���������϶��ĹؽڽǶ�
hold on;

%% ���������ռ��ɢ��ͼ
N=60000;    %�������
zero_N = zeros(N,1);
theta1=(limitmin_1+(limitmax_1-limitmin_1)*rand(N,1))*pi/180; %�ؽ�1����
theta2=(limitmin_2+(limitmax_2-limitmin_2)*rand(N,1))*pi/180; %�ؽ�2����
theta3=(limitmin_3+(limitmax_3-limitmin_3)*rand(N,1))*pi/180; %�ؽ�3����

qq=[theta1,theta2,theta3,zero_N,zero_N,zero_N];

Mricx=puma.fkine(qq); %��ÿһ��λ�ý����˶�ѧ����toolλ��

x=reshape(Mricx(1,4,:),N,1);
y=reshape(Mricx(2,4,:),N,1);
z=reshape(Mricx(3,4,:),N,1);

%��̬������ʾ
% for i=1:1:N
%     puma.plot([x(i),y(i),z(i),zero_N(i),zero_N(i),zero_N(i)]);
%     plot3(x(i),y(i),z(i),'b.','MarkerSize',0.5);%�������
%     hold on;
% end

plot3(x,y,z,'b.','MarkerSize',0.5);%�������


%% ���˶�ѧ���ؽڽ�
qz = [0 0 0 0 0 0];
T1 = puma.fkine(qz);
X0 = T1(1,4);Y0 = T1(2,4);Z0 = T1(3,4);

% �������λ�˾���
X = 0.5; Y = 0.5; Z = 0.8;
T2 = [1,0,0,X;
    0,1,0,Y;
    0,0,1,Z;
    0,0,0,1];
joint_angel = puma.ikine6s(T2,'ru');

%% Բ���켣�滮

fprintf("Բ���켣�滮 \n");
% ����ʱ������
endtime = 2;
t = (0 :0.05 :endtime)';
Nn = (endtime / 0.05) + 1;
% ���ɹ켣�滮
[theta,thetad,thetadd] = jtraj(qz,joint_angel,t);  % �Ľ�����ζ���ʽ��ֵ
figure(2);
for i= 1:1:Nn
    puma.plot(theta(i,:));%������ʾ
    Tra=puma.fkine(theta(i,:));
    plot3(Tra(1,4),Tra(2,4),Tra(3,4),'b.','MarkerSize',5);%�������
    title("Բ���켣�滮");
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
legend('�Ƕ�','�ٶ�','���ٶ�');title('�ؽ�1');
grid on;

subplot(312);
plot(t,theta(:,2),'LineWidth',2);hold on;
plot(t,thetad(:,2),'LineWidth',2);hold on;
plot(t,thetadd(:,2),'LineWidth',2);
legend('�Ƕ�','�ٶ�','���ٶ�');title('�ؽ�2');
grid on;

subplot(313);
plot(t,theta(:,3),'LineWidth',2);hold on;
plot(t,thetad(:,3),'LineWidth',2);hold on;
plot(t,thetadd(:,3),'LineWidth',2);
legend('�Ƕ�','�ٶ�','���ٶ�');title('�ؽ�3');
grid on;

figure(4);
subplot(311);
plot(t,theta(:,4),'LineWidth',2);hold on;
plot(t,thetad(:,4),'LineWidth',2);hold on;
plot(t,thetadd(:,4),'LineWidth',2);
legend('�Ƕ�','�ٶ�','���ٶ�');title('�ؽ�4');
grid on;
subplot(312);
plot(t,theta(:,5),'LineWidth',2);hold on;
plot(t,thetad(:,5),'LineWidth',2);hold on;
plot(t,thetadd(:,5),'LineWidth',2);
legend('�Ƕ�','�ٶ�','���ٶ�');title('�ؽ�5');
grid on;
subplot(313);
plot(t,theta(:,6),'LineWidth',2);hold on;
plot(t,thetad(:,6),'LineWidth',2);hold on;
plot(t,thetadd(:,6),'LineWidth',2);
legend('�Ƕ�','�ٶ�','���ٶ�');title('�ؽ�6');
grid on;


%% ֱ�߹켣�滮
% ֱ�ߵȼ����ֵ��
fprintf("ֱ�߹켣�滮 \n");
l = X-X0; m = Y-Y0; n = Z-Z0;
fprintf("\n�ռ�ֱ�߷���Ϊ��  x - %f / %f =  y- %f / %f =  z - %f / %f\n",X0,l,Y0,m,Z0,m);

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
    puma.plot(theta_LineTraj(i,:));%������ʾ
    LineTraj=puma.fkine(theta_LineTraj(i,:));
    plot3(LineTraj(1,4),LineTraj(2,4),LineTraj(3,4),'b.','MarkerSize',5);%�������
    hold on;
    title("ֱ��·���滮");
end


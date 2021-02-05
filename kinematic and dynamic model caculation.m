% innfos3.m
% 单臂动力学结构参数
%先定义运动学参数
%全部参数化
%syms d1 d2 d3;暂时删掉
% d=[       0.0792,   0.0792,    0.03350];%还未测量！！！！最初始
d=[       0.0754,   0.0754,    0.034];%进行了简化
a=[        0,    0,     0];%/1000
alpha=[ pi/2, pi/2,     0];
offset=[pi,  -pi/2,     0];

%使用offset 配置运动学参数
L(1)=Link('d',d(1),'a',a(1),'alpha',alpha(1),'offset',offset(1)); 
L(2)=Link('d',d(2),'a',a(2),'alpha',alpha(2),'offset',offset(2));
L(3)=Link('d',d(3),'a',a(3),'alpha',alpha(3));

du=pi/180;
ra=180/pi;
%定义关节范围
L(1).qlim =[-170, 170]*du;
L(2).qlim =[-170, -170]*du;%-10,130
L(3).qlim =[-170,-170]*du;%-140,0

bot=SerialLink(L,'name','innfos末端姿态稳定');
%bot.tool= transl(0, 0, tool)

% 动力学参数，数据可来源于SolidWorks，但要注意单位，注意！m3表示末端负载
syms I1xx I1yy  I1zz  I1xy  I1xz  I1yz  xc1  yc1  zc1  m1 ;
syms I2xx  I2yy  I2zz  I2xy  I2xz  I2yz  xc2   yc2  zc2  m2 ;
syms I3xx  I3yy  I3zz  I3xy  I3xz  I3yz  xc3  yc3  zc3  m3 data;
data=[
    %     Ixx，    Iyy,      Izz,        Ixy,        Ixz,        Iyz,          xc,         yc,        zc,       m      xp    yp    zp

       2152.449,  2095.640,  206.630,      0,          0,    -71.267,          0,          0,      75.4,    0.381,    0,   0,    0;
       2096.966,  2153.820,  206.676,      0,     71.943,          0,          0,          0,      75.4,    0.381,    0,   0,    0;
       4625.752,  4625.752,  110.689,      0,          0,          0,          0,          0,     120.4,    0.290,    0,   0,    86;
    ];
%% data=[
    %     Ixx，    Iyy,      Izz,        Ixy,        Ixz,        Iyz,         xc,         yc,        zc,       m
%       2152.449,  2095.640,  206.630,      0,          0,    -71.267,          0,     -2.450,      70.421,     0.381;
%       2096.966,  2153.820,  206.676,      0,     71.943,          0,      2.474,          0,      70.445,     0.381;
%       4625.752,  4625.752,  110.689,      0,          0,          0,          0,          0,     120.399,     0.290;
%    ];

data(:,1:6)=data(:,1:6)./1000000;
data(:,7:9)=data(:,7:9)./1000;
data(:,11:13)=data(:,11:13)./1000;
% 逆动力学求解函数
% 输入 机械臂名称，位置、速度、加速度矩阵
% 输出关节扭矩
% 是MDH_Dy.m的改进版，使用offset
% 改进体现在直接使用机器人的某些参数


%% 定义各种变量   
% 定义关节角度
    syms q1 q2 q3;
    syms dq1 dq2 dq3;
    syms ddq1 ddq2 ddq3;
   % [q1,q2,q3]=deal(Q(1),Q(2),Q(3));
   % [dq1,dq2,dq3]=deal(DQ(1),DQ(2),DQ(3));
   %[ddq1,ddq2,ddq3]=deal(DDQ(1),DDQ(2),DDQ(3));
   %%定义D，T
% 创建数组和变量
syms Dij Dijj Dijk Dijk
    Dij=cell(3,3);
    Dijj=cell(3,3);
    Dijk=cell(5,3);
    Di=cell(5,1);
    q=[q1;q2;q3];
    dq=[dq1;dq2;dq3];
    ddq=[ddq1;ddq2;ddq3];
    dqdq=[dq1*dq2;dq1*dq3;
        dq2*dq3];
    %% 计算伪惯量矩阵
    J_cell=cell(3,1);
    %Ixx，Iyy,Izz,  Ixy,Ixz,Iyz,  xc,yc,zc,m
    %长度单位mm，惯性张量kg*mm，
   % data=[
    %     Ixx，    Iyy,      Izz,        Ixy,        Ixz,        Iyz,         xc,         yc,        zc,       m
 %      47.316,  51.601,   77.113,     -0.003,     -2.549,     -0.016,     -0.598,      0.016,   -23.413,   0.076;
 %      62.746, 651.130,  704.486,     29.632,     -0.001,     -0.003,    104.910,    -31.512,     0.001,   0.151;
 %       6.264, 224.674,  228.590,    -14.345,     -0.006,          0,     69.863,      8.061,     0.015,   0.065;
       
 %   ];

%     data=[
%         22.134, 30.762, 24.755, -2.241, -0.00, 0, -2.546, -21.352, 0.302, -2.948;
%         85.387, 822.001, 893.708, 48.758, 0, 0, 102.348, -34.530, 0, 0.223;
%         5.440, 281.010, 283.608,  -17.983, 0, 0, 68.088, 7.699, 0, 0.084;
%        
%         ];
    for i=1:3
        Ixx=data(i,1);Iyy=data(i,2);Izz=data(i,3);
        Ixy=data(i,4);Ixz=data(i,5);Iyz=data(i,6);
        xc=data(i,7);yc=data(i,8);zc=data(i,9);
        m=data(i,10);

        J=[(-Ixx+Iyy+Izz)/2,Ixy,Ixz,m*xc;
            Ixy,(Ixx-Iyy+Izz)/2,Iyz,m*yc;
            Ixz,Iyz,(Ixx+Iyy-Izz)/2,m*zc;
            m*xc,m*yc,m*zc,m];

        J_cell{i}=J;   
    end
    %% 运动学计算转移矩阵
     global T_cell;
    T_cell=cell(3,1);
 

    for i=1:3
        T_cell{i}=[ cos(q(i)+offset(i)), -sin(q(i)+offset(i))*cos(alpha(i)), sin(q(i)+offset(i))*sin(alpha(i)), a(i)*cos(q(i)+offset(i));
 sin(q(i)+offset(1)),  cos(q(i)+offset(i))*cos(alpha(i)), -cos(q(i)+offset(i))*sin(alpha(i)),   a(i)*sin(q(i)+offset(i));
      0,        sin(alpha(i)), cos(alpha(i)), d(i);
     0,        0, 0,  1];
% T_cell{2}=[ cos(q2 - pi/2), 0,   sin(q2 - pi/2),  0;
% sin(q2 - pi/2),            0,  -cos(q2 - pi/2),  0;
 %             0,                  1, 0, d(2);
 %            0,                  0, 0,  1];
 %T_cell{3}=[ cos(q3), -sin(q3), 0,  0;
 %sin(q3),  cos(q3), 0,  0;
 %      0,        0, 1, d(3);
 %    0,        0, 0,  1];
 
    end
    

    %% Dij
    %行
    for i=1:3
        %列
        for j=1:3
            p=max(i,j);
            %累加
            D=0;
            for pp=p:3          
                Upj=Uij(pp,j);
                Upi=Uij(pp,i);
                D=D+trace(Upj*J_cell{pp}*Upi.');           
            end
            Dij{i,j}=simplify(D);
        end
    end
    %% Dijj
    for i=1:3
        for j=1:3
            p=max(i,j);
            %累加
            D=0;
            for pp=p:3          
                Upjj=Uijk(pp,j,j);
                Upi=Uij(pp,i);
                D=D+trace(Upjj*J_cell{pp}*Upi.');           
            end
            Dijj{i,j}=simplify(D);
        end
    end
    %% Dijk
    %标记标号j和k，for循环记录不太方便，所以直接写下来
    dijk_j=[1,1,2];
    dijk_k=[2,3,3];
    %行
    for i=1:3
        %列循环
        for s=1:3
            %列内标号循坏
            j=dijk_j(1,s);
            k=dijk_k(1,s);
            %p=max(i,j,k)
            p=max(i,j);
            p=max(p,k);
            %累加
            D=0;
            for pp=p:3
              Upjk=Uijk(pp,j,k);
              Upi=Uij(pp,i);
              D=D+trace(Upjk*J_cell{pp}*Upi.');  
            end
            Dijk{i,s}=simplify(D);
        end
    end
    %% Di
g=[0,-9.81,0,0];%空中机械臂重力与坐标系方向一致，所以为正

    for i=1:3
        D=0;
        %累加
        for p=i:3
           m_p=data(p,10);
           %位置和加速度都是齐次
           
           Upi=Uij(p,i);
           %位置是在p坐标系下 r是1×4的列矩阵
           r_cp=[data(p,11);data(p,12);data(p,13);1];
           D=D+m_p*g*Upi*r_cp; %前两个关节力矩都为0，Up2只在后两行为0，g只在第三行不为0
           
        end
        Di{i}=-simplify(D);
    end
     %% 计算动力学方程，不加；
     T=cell(3,1);
     for i=1:3
    T{i}= [Dij{i,1},Dij{i,2},Dij{i,3}]*ddq + [Dijj{i,1},Dijj{i,2},Dijj{i,3}]*(dq.^2) + 2*[Dijk{i,1},Dijk{i,2},Dijk{i,3}]*dqdq +Di{i};
    T{i}=vpa(simplify( T{i}),3);%简化变为分数
     end
    %%
% 计算拉格朗日动力学参数Uij
% 在MDH_Dy中使用
function [ U ] = Uij( i,j )
    global T_cell;
%旋转矩阵对角度求导
   Q=[0, -1, 0, 0;
      1,  0, 0, 0;
      0,  0, 0, 0;
      0,  0, 0, 0];
   U=1;
   for kk=1:j-1
        U=U*T_cell{kk};
    end
    
    U=U*Q;
    
    for kk=j:i
        U=U*T_cell{kk};
    end
end
% 计算拉格朗日动力学参数Uij
% 在MDH_Dy中使用
%%
function [ U ] = Uijk(  i,j,k )
   global T_cell;
   Q=[0, -1, 0, 0;
       1,  0, 0, 0;
       0,  0, 0, 0;
       0,  0, 0, 0];
   U=1;
   for p=1:j-1
        U=U*T_cell{p};
   end
    
   U=U*Q;
   %for先判断再循坏 
   for p=j:k-1
        U=U*T_cell{p};
   end
    
   U=U*Q;
    
   for p=k:i
        U=U*T_cell{p};
   end

end

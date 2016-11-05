clear all;
clc;
syms alpha beta gamma t_x t_y t_z x_l y_l z_l

ROT_X = [ 1,0,0 ; 0,cos(gamma),-sin(gamma);  0,sin(gamma),cos(gamma)];
ROT_Y = [cos(beta),0,sin(beta);  0,1,0 ; -sin(beta),0,cos(beta)];
ROT_Z = [ cos(alpha),-sin(alpha),0; sin(alpha),cos(alpha),0 ; 0,0,1];
%Total rotation for Kuka
R_CAM = ROT_Z*ROT_Y*ROT_X;
T_CAM = [t_x;t_y;t_z];                       %Translation Matrix
%Complete extermnal matrix with rotation and translation  
CE=[R_CAM,T_CAM;0 0 0 1];


% Las = [528.97,140.74,212.07,-53.98,-1.14,179.92;
%        644.93,170.72,185.56,-53.98,-1.14,179.92;
%        364.54,171.83,209.80,-53.98,-1.14,179.92;
%        ];
Las = [47.73,100.72,140.83,-54.55,-1.50,-176.64;
    -73.51,138.01,216.47,-54.55,-1.50,-176.64;
    -2.25,165.54,208.93,-54.55,-1.50,-176.64;
    108.89,162.13,168.33,-54.55,-1.50,-176.64;
    59.06,191.57,199.13,-54.55,-1.50,-176.64;
    93.47,188.64,184.11,-54.55,-1.50,-176.64;
       
       ];
    
angles=size(Las,1);

recx = Las(:,1);%169;%182.96;%182.45;%3 pellet182.19;%92.50;
recy = Las(:,2);%212;%200.70;%3 pellet 212.37;%237.48;
recz = Las(:,3);%256;%158.80;%3 pellet 157.25;
reca = Las(:,4);
recb = Las(:,5);
recc = Las(:,6);


% world_data_p=[560.54,44.69,112.3,1;
%             559.66,65.72,155.42,1;
%             674.73,93.95,155.89,1;
%             675.54,72.87,113.0,1;
%             394.52,73.67,153.74,1;
%             395.18,52.81,110.80,1;
%             ];
world_data_p=[78.51,-10.15,145.20,1;
            -37.72,46.05,144.84,1;
            33.67,62.08,145.44,1;
            140.95,51.06,146.34,1;
           93.74,96.81,146.18,1;
            126.58,88.74,146.10,1;
            ];
% laser_data=[15.6,0,227.69,1;
%             34.615,0,271.9,1;
%             31.370,0,298.795,1;
%             12.10,0,254.262,1;
%             11.620,0,271.385,1;
%             -7.310,0,227.135,1;
%             ];
        
laser_data=[-4.45,0,335.72,1;
            16.415,0,259.140,1;
            2.570,0,267.060,1;
            -3.570,0,310.380,1;
            13.040,0,277.81,1;
            8.075,0,292.900,1;
            ];
world_data=[];
 %laser_data=[];
 for frame = 1:6
   

    Rx(:,:,frame)=[1 0 0;0 cosd(recc(frame)) -sind(recc(frame));0 sind(recc(frame)) cosd(recc(frame))];
    Ry(:,:,frame)=[cosd(recb(frame)) 0 sind(recb(frame)); 0 1 0;-sind(recb(frame)) 0 cosd(recb(frame))];
    Rz(:,:,frame)=[cosd(reca(frame)) -sind(reca(frame)) 0;sind(reca(frame)) cosd(reca(frame)) 0;0 0 1];
    Q(:,:,frame)=Rz(:,:,frame)*Ry(:,:,frame)*Rx(:,:,frame);
    T(:,:,frame)=[recx(frame);recy(frame);recz(frame)];
    B_T_T(:,:,frame)=[Q(:,:,frame) T(:,:,frame);0 0 0 1];%Base with respt to tool matrix
    
 for point = 1
    world_data = [world_data;(B_T_T(:,:,frame)\(world_data_p((frame-1)*1+point,:)'))'];
 end
 
    %laser_data = [laser_data;pnt_bs(:,[5 6])];
end  
%calib=CI*ext ;        
%C_T_B  =  CE * inv(B_T_T);

T_T_L = CE ;
coord =  [x_l;y_l;z_l;1];
transCoord = T_T_L*coord;

x1 = transCoord(1);
y1 = transCoord(2);
z1 = transCoord(3);

u= transCoord(1:3);

j1=jacobian(u,[alpha;beta;gamma;t_x;t_y;t_z]);
opt_matrix = @(alpha, beta, gamma ,t_x, t_y, t_z,x_l ,y_l,z_l,j1)subs(j1);
%opt_du = @(alpha, beta, gamma ,t_x, t_y, t_z,x_l ,y_l,z_l,u1,v1,u)subs(u-[u1;v1]);

opt_du = @(alpha, beta, gamma ,t_x, t_y, t_z,x_l ,y_l,z_l,x1,y1,z1,u)subs(u-[x1;y1;z1]);

%for substituting values
%R=[0.573382 -0.819280 0.003799; 0.819229 0.573390 0.009343; -0.009833 -0.002245 0.9999];
%T=[10.503989;-65.653220;128.189572];
%%QSensorTCP=[-0.8232 -0.5674 -0.0202 114.0241;-0.5665  0.8233 -0.0370 45.605; 0.0377 -0.0190 -0.9991 340.2798; 0 0 0 1];
%QCameraTCP=[R T;0 0 0 1];
%%QCameraLaser=QCameraTCP*QSensorTCP ; 

QLaserTCP = [-0.7986,   -0.6007,   -0.0370,  113.9513;
    	-0.6004,    0.7995,   -0.0201,   45.4287;
		 0.0416,    0.0061,   -0.9991,  338.8648;
		 0,         0,         0,    1.0000;];
QCameraLaser=QLaserTCP;
beta1=atan2(-QCameraLaser(3,1),sqrt(QCameraLaser(1,1)^2+QCameraLaser(2,1)^2)) ;                     %Yaw
alpha1=atan2(QCameraLaser(2,1)/cos(beta1),QCameraLaser(1,1)/cos(beta1)) ;                    %Roll
gamma1=atan2(QCameraLaser(3,2)/cos(beta1),QCameraLaser(3,3)/cos(beta1));                     %Pitch
t_x1=QCameraLaser(1,4);
t_y1=QCameraLaser(2,4);
t_z1=QCameraLaser(3,4);

% alpha1 = 4.05;
% beta1=-0.03;
% gamma1=-0.058;
% t_x1= 19.69;%3.87;
% t_y1=67.01;%78.63;
% t_z1=114.1;

w_x = world_data(:,1);
w_y = world_data(:,2);
w_z = world_data(:,3);
 
u_x=laser_data(:,1);
u_y=laser_data(:,2);
u_z=laser_data(:,3);

 for frame=1:10
     
 er(frame)=0;
 init_optim=[];
 U_init=[];
 
 for point=1:size(world_data,1)
    init_optim = [init_optim;opt_matrix(alpha1,beta1,gamma1,t_x1,t_y1,t_z1,w_x(point), w_y(point), w_z(point),j1)];
    U_init = [U_init;opt_du(alpha1,beta1,gamma1,t_x1,t_y1,t_z1,w_x(point),w_y(point), w_z(point),u_x(point),u_y(point),u_z(point),u)];
    er = er + norm(opt_du(alpha1,beta1,gamma1,t_x1,t_y1,t_z1,w_x(point),w_y(point), w_z(point),u_x(point),u_y(point),u_z(point),u));
 end
     
 epsilon=-pinv(init_optim)*U_init;
 %epsilon = epsilon;
 beta1=beta1+epsilon(2);
 alpha1=alpha1+epsilon(1);
 gamma1=gamma1+epsilon(3);
 t_x1=t_x1+epsilon(4);
 t_y1=t_y1+epsilon(5);
 t_z1=t_z1+epsilon(6);
 er(frame)/size(world_data,1)
  
 end
 
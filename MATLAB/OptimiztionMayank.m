clear all;
clc;
syms alpha beta gamma t_x t_y t_z x_l y_l z_l
pnt_arr=load('newcalibdata18.txt');


%CI=[2297.499276025967 0  1273.469934600958;0 2298.113521619946  998.0020028393124 ;0 0 1];     %Internal Paramter
%CI=[2227.308888 0.000000 1143.948131; 0.000000 2226.379210 1016.562688; 0.000000 0.000000 1.000000]; % with distortion
%CI=[2176.211068 0.000000 1244.289249; 0.000000 2178.633804 1041.836938; 0.000000 0.000000 1.000000]; % without Distortion
CI=[2328.556745 0.000000 1254.058997; 0.000000 2334.311366 1012.072590; 0.000000 0.000000 0.999071]
ROT_X = [ 1,0,0 ; 0,cos(gamma),-sin(gamma);  0,sin(gamma),cos(gamma)];
ROT_Y = [cos(beta),0,sin(beta);  0,1,0 ; -sin(beta),0,cos(beta)];
ROT_Z = [ cos(alpha),-sin(alpha),0; sin(alpha),cos(alpha),0 ; 0,0,1];
%Total rotation for Kuka
R_CAM = ROT_Z*ROT_Y*ROT_X;
T_CAM = [t_x;t_y;t_z];                       %Translation Matrix
%Complete extermnal matrix with rotation and translation  
CE=[R_CAM,T_CAM;0 0 0 1];


% cam = [40.93,-8.04,-189.78,52.99,0.82,-0.73;
%         32.22,-7.56,-189.92,54.23,11.31,14.00;
%         173.19,-11.11,-189.51,57.15,-16.25,-25.89;
%         147.45,98.72,-191.65,55.98,-35.71,-4.79;
%         45.63,-67.54,-176.28,52.53,12.06,-9.64];
% cam = [39.78,-8.74,-181.87,52.99,0.82,-0.73;
%         32.22,-7.56,-189.92,54.23,11.31,14.00;
%         168.23,-11.78,-183.23,57.15,-16.25,-25.89;
%         143.39,94.60,-186.09,55.98,-35.78,-4.79;
%         44.45,-66.27,-168.44,52.53,12.06,-9.64];
cam = [72.98, 47.79, 141.73, -54.55, -1.46, 177.68;
         59.33,64.31,144.77,-54.65,2.37,172.98;
         56.92,94.40,152.49,-54.66,-2.93,-177.67;
         47.73,100.72,140.83,-54.55,-1.50,-176.64];
         
    
    
    
angles=size(cam,1);

recx = cam(:,1);%169;%182.96;%182.45;%3 pellet182.19;%92.50;
recy = cam(:,2);%212;%200.70;%3 pellet 212.37;%237.48;
recz = cam(:,3);%256;%158.80;%3 pellet 157.25;
reca = cam(:,4);
recb = cam(:,5);
recc = cam(:,6);


world_data=[];
camera_data=[];

for i = [1 2 3 4]
    
    pnt_bs=[];
    for j=1:5:size(pnt_arr,2)/angles
        pnt_bs=[pnt_bs;[pnt_arr((((i-1)*size(pnt_arr,2)/angles)+j):(((i-1)*size(pnt_arr,2)/angles)+j+2)),1,pnt_arr((((i-1)*size(pnt_arr,2)/angles)+j+3):(((i-1)*size(pnt_arr,2)/angles)+j+4))]];
    end
    
    Rx(:,:,i)=[1 0 0;0 cosd(recc(i)) -sind(recc(i));0 sind(recc(i)) cosd(recc(i))];
    Ry(:,:,i)=[cosd(recb(i)) 0 sind(recb(i)); 0 1 0;-sind(recb(i)) 0 cosd(recb(i))];
    Rz(:,:,i)=[cosd(reca(i)) -sind(reca(i)) 0;sind(reca(i)) cosd(reca(i)) 0;0 0 1];
    Q(:,:,i)=Rz(:,:,i)*Ry(:,:,i)*Rx(:,:,i);
    T(:,:,i)=[recx(i);recy(i);recz(i)];
    B_T_T(:,:,i)=[Q(:,:,i) T(:,:,i);0 0 0 1];%Base with respt to tool matrix

    world_data = [world_data;(B_T_T(:,:,i)\(pnt_bs(:,[1 2 3 4])'))'];
    camera_data = [camera_data;pnt_bs(:,[5 6])];
end


    
%calib=CI*ext ;        
%C_T_B  =  CE * inv(B_T_T);

calib = CE ;
coord =  [x_l;y_l;z_l;1];
transCoord = calib*coord;

u1=transCoord(1)/transCoord(3);    v1=transCoord(2)/transCoord(3);
      
projCoord(1) = u1; 
projCoord(2) = v1;
projCoord(3) = 1;
 
% %k1,k2 p1 p2 k3 k4 k5 k6
% k1 = 0.267258;
% k2 = -7.730646 ;
% p1 = 0.004472 ;
% p2 = -0.005795 ;
% k3 = 35.693519 ;
% k4 = 0;
% k5 = 0;
% k6 = 0;

k1 = -0.201788;
k2 = 0;
p1 = 0 ;
p2 = 0 ;
k3 = 0 ;
k4 = 0;
k5 = 0;
k6 = 0;

% %[-0.060870 79.204406 0.005184 0.004357 2309.438712 0.222109 73.118471 2414.666642] 
% %  New
% % k1 = -0.060870;
% % k2 = 79.204406;
% % p1 = 0.005184;
% % p2 = 0.004357;
% % k3 = 2309.438712;
% % k4 = 0.222109;
% % k5 = 73.118471;
% % k6 = 2414.666642;
% % 
r(1) = projCoord(1);
r(2) = projCoord(2);
r(3) = projCoord(1).*projCoord(1) + projCoord(2).*projCoord(2);
r(4) = (1 + (k1*r(1)) + (k2* r(1).*r(1)) + (k3* r(1).*r(1).* r(1)));
r(4) = r(4)./(1 + (k4* r(1)) + (k5* r(1).*r(1)) + (k6* r(1).*r(1).* r(1)));

distortion(1) = r(1).* r(4) + 2*p1*r(1).*r(2) + p2*(r(3) + 2*r(1).*r(1));
distortion(2)  = r(2).* r(4) + 2*p2*r(1).*r(2) + p1*(r(3) + 2*r(2).*r(2));
distortion(3) = 1;

distortion_free = (CI * projCoord')';
  
u1=distortion_free(1)/distortion_free(3);
v1=distortion_free(2)/distortion_free(3); 

u=[u1;v1];
         
j1=jacobian(u,[alpha;beta;gamma;t_x;t_y;t_z]);
opt_matrix = @(alpha, beta, gamma ,t_x, t_y, t_z,x_l ,y_l,z_l,j1)subs(j1);
opt_du = @(alpha, beta, gamma ,t_x, t_y, t_z,x_l ,y_l,z_l,u1,v1,u)subs(u-[u1;v1]);

%for substituting values
%R=[0.573382 -0.819280 0.003799; 0.819229 0.573390 0.009343; -0.009833 -0.002245 0.9999];
%T=[10.503989;-65.653220;128.189572];
%%QSensorTCP=[-0.8232 -0.5674 -0.0202 114.0241;-0.5665  0.8233 -0.0370 45.605; 0.0377 -0.0190 -0.9991 340.2798; 0 0 0 1];
%QCameraTCP=[R T;0 0 0 1];
%%QCameraLaser=QCameraTCP*QSensorTCP ;    
%QCameraLaser=QCameraTCP;
%beta1=atan2(-QCameraLaser(3,1),sqrt(QCameraLaser(1,1)^2+QCameraLaser(2,1)^2)) ;                     %Yaw
%alpha1=atan2(QCameraLaser(2,1)/cos(beta1),QCameraLaser(1,1)/cos(beta1)) ;                    %Roll
%gamma1=atan2(QCameraLaser(3,2)/cos(beta1),QCameraLaser(3,3)/cos(beta1));                     %Pitch
%t_x1=QCameraLaser(1,4);
%t_y1=QCameraLaser(2,4);
%t_z1=QCameraLaser(3,4);

% alpha1 = 4.058;
% beta1=-0.047;
% gamma1=-0.08;
beta1 = -0.0382;
alpha1 = -2.2215;
gamma1= -0.0271
t_x1= 11.039197  ;%3.87;
t_y1=65.119311;%68.65;%78.63;
t_z1=133.896502%17.48;%123.44;

w_x = world_data(:,1);
w_y = world_data(:,2);
w_z = world_data(:,3);
 
u_x=camera_data(:,1);
u_y=camera_data(:,2);

 for i=1:10
     
 er(i)=0;
 init_optim=[];
 U_init=[];
 
 for j=1:size(world_data,1)
    init_optim = [init_optim;opt_matrix(alpha1,beta1,gamma1,t_x1,t_y1,t_z1,w_x(j), w_y(j), w_z(j),j1)];
    U_init = [U_init;opt_du(alpha1,beta1,gamma1,t_x1,t_y1,t_z1,w_x(j),w_y(j), w_z(j),u_x(j),u_y(j),u)];
    er = er + norm(opt_du(alpha1,beta1,gamma1,t_x1,t_y1,t_z1,w_x(j),w_y(j), w_z(j),u_x(j),u_y(j),u));
 end
     
 epsilon=-pinv(init_optim)*U_init;
 epsilon = epsilon;
 beta1=beta1+epsilon(2);
 alpha1=alpha1+epsilon(1);
 gamma1=gamma1+epsilon(3);
 t_x1=t_x1+epsilon(4);
 t_y1=t_y1+epsilon(5);
 t_z1=t_z1+epsilon(6);
 er(i)/size(world_data,1)
  
 end
 
%x1=laser_opti;
x1=load('8.txt');

%x1=load('C:\Users\Pranav\Desktop\1.txt');
%x1= laser_data;
%x1=x1/1000;
% x1=LaserProjection1
x2=zeros(size(x1));
x2(:,1)=x1(:,1).*1000;
x2(:,2)=x1(:,2).*1000 ;
x2(:,3)=x1(:,3).*1000;
x2(:,4)=1;
x2(:,1)=x2(:,1)- (0*x2(:,4));
x2(:,2)=x2(:,2)+ (0*x2(:,4));
r=zeros(size(x1));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   convert TCP to Base frame by using data from robot
%   CHANGE VALUES FOR EVERY SCAN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tx=528.96;%169;%182.96;%182.45;%3 pellet182.19;%92.50;
ty=140.73;%212;%200.70;%3 pellet 212.37;%237.48;
tz=212.08;%256;%158.80;%3 pellet 157.25;
a=-53.98;
b=-1.13;
c=179.92;

%   calc of R|T matrix
Rx=[1 0 0;0 cosd(c) -sind(c);0 sind(c) cosd(c)];
Ry=[cosd(b) 0 sind(b); 0 1 0;-sind(b) 0 cosd(b)];
Rz=[cosd(a) -sind(a) 0;sind(a) cosd(a) 0;0 0 1];
Q=Rz*Ry*Rx;
T=[tx;ty;tz];
B_T_T=[Q T;0 0 0 1];%Base with respt to tool matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Old New CI=[2354.862624 0.000000 1285.167079; 0.000000 2355.274166 1050.767930; 0.000000 0.000000 1];
ROT_Y = [cos(beta1),0,sin(beta1);  0,1,0 ; -sin(beta1),0,cos(beta1)];
ROT_X = [ 1,0,0 ; 0,cos(gamma1),-sin(gamma1);  0,sin(gamma1),cos(gamma1)];
ROT_Z = [ cos(alpha1),-sin(alpha1),0; sin(alpha1),cos(alpha1),0 ; 0,0,1];
R_KUKA = ROT_Z*ROT_Y*ROT_X;
t_new=[t_x1,t_y1,t_z1];
C_T_T=[R_KUKA,t_new';0 0 0 1]
inv(C_T_T)
C_T_B=C_T_T/(B_T_T);
projection=C_T_B(1:3,1:4)

base_to_image=size(x2,1);
for i=1:size(x2,1)
base_to_image(i,1:3)=projection*x2(i,:)';
end
Trans_Laser_new=base_to_image;
Trans_Laser_new(:,1)=Trans_Laser_new(:,1)./Trans_Laser_new(:,3);
Trans_Laser_new(:,2)=Trans_Laser_new(:,2)./Trans_Laser_new(:,3);
Trans_Laser_new(:,3) = Trans_Laser_new(:,3)./Trans_Laser_new(:,3);

% %k1,k2 p1 p2 k3 k4 k5 k6
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
% r(:,1) = Trans_Laser_new(:,1);
% r(:,2) = Trans_Laser_new(:,2);
% r(:,3) = Trans_Laser_new(:,1).*Trans_Laser_new(:,1) + Trans_Laser_new(:,2).*Trans_Laser_new(:,2);
% r(:,4) = (1 + (k1*r(:,1)) + (k2* r(:,1).*r(:,1)) + (k3* r(:,1).*r(:,1).* r(:,1)));
% r(:,4) = r(:,4)./(1 + (k4* r(:,1)) + (k5* r(:,1).*r(:,1)) + (k6* r(:,1).*r(:,1).* r(:,1)));
% 
% Trans_Laser_new(:,1) = r(:,1).* r(:,4) + 2*p1*r(:,1).*r(:,2) + p2*(r(:,3) + 2*r(:,1).*r(:,1));
% Trans_Laser_new(:,2)  = r(:,2).* r(:,4) + 2*p2*r(:,1).*r(:,2) + p1*(r(:,3) + 2*r(:,2).*r(:,2));

distortion_free = (CI * Trans_Laser_new')';

figure(2)
image=imread('8.bmp');
imshow(image)
hold on
scatter(distortion_free(:,1),distortion_free(:,2),'r','.')

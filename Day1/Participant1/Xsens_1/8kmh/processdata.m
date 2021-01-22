clear all
close all
rand('state',0)
randn('state',0)

% acc & gyro data gait1
IndoorGait1KneeLeft.raw{1}=importdata('accelerometer and gyro data/Indoor/Gait/gait1_accel_gyro_00341961_left_thigh.txt');
IndoorGait1KneeLeft.raw{2}=importdata('accelerometer and gyro data/Indoor/Gait/gait1_accel_gyro_00341962_left_shank.txt');

% Rotation matrices static 
IndoorGait1KneeLeft.thigh_left_R = getrotmatrices('rotational matrices/Indoor/Gait/gait1_rot_mat_00341961_left_thigh.txt');
IndoorGait1KneeLeft.shank_left_R = getrotmatrices('rotational matrices/Indoor/Gait/gait1_rot_mat_00341962_left_shank.txt');

IndoorGait1KneeLeft.n=length(IndoorGait1KneeLeft.raw{1}.data);
IndoorGait1KneeLeft.time=(IndoorGait1KneeLeft.raw{1}.data(:,1)-IndoorGait1KneeLeft.raw{1}.data(1,1)).*(1/60);

IndoorGait1KneeLeft.a(:,:,1)=IndoorGait1KneeLeft.raw{1}.data(:,3:5);
IndoorGait1KneeLeft.a(:,:,2)=IndoorGait1KneeLeft.raw{2}.data(:,3:5);

figure
plot(IndoorGait1KneeLeft.a(:,1,1),'r')
hold on
plot(IndoorGait1KneeLeft.a(:,2,1),'g')
plot(IndoorGait1KneeLeft.a(:,3,1),'b')
plot(IndoorGait1KneeLeft.a(:,1,2),'r--')
plot(IndoorGait1KneeLeft.a(:,2,2),'g--')
plot(IndoorGait1KneeLeft.a(:,3,2),'b--')
for i=1:3
    for j=1:2
IndoorGait1KneeLeftFiltered.a(:,i,j)=lowpass(IndoorGait1KneeLeft.a(:,i,j),2.5,1);
    end
end
plot(IndoorGait1KneeLeftFiltered.a(:,1,1),'r','LineWidth',2)
hold on
plot(IndoorGait1KneeLeftFiltered.a(:,2,1),'g','LineWidth',2)
plot(IndoorGait1KneeLeftFiltered.a(:,3,1),'b','LineWidth',2)
plot(IndoorGait1KneeLeftFiltered.a(:,1,2),'r--','LineWidth',2)
plot(IndoorGait1KneeLeftFiltered.a(:,2,2),'g--','LineWidth',2)
plot(IndoorGait1KneeLeftFiltered.a(:,3,2),'b--','LineWidth',2)


IndoorGait1KneeLeft.g(:,:,1)=IndoorGait1KneeLeft.raw{1}.data(:,6:8);
IndoorGait1KneeLeft.g(:,:,2)=IndoorGait1KneeLeft.raw{2}.data(:,6:8);
% IndoorGait1KneeLeft.g(:,:,1)=flipud(IndoorGait1KneeLeft.g(:,:,1));

figure
plot(IndoorGait1KneeLeft.g(:,1,1),'r')
hold on
plot(IndoorGait1KneeLeft.g(:,2,1),'g')
plot(IndoorGait1KneeLeft.g(:,3,1),'b')
plot(IndoorGait1KneeLeft.g(:,1,2),'r--')
plot(IndoorGait1KneeLeft.g(:,2,2),'g--')
plot(IndoorGait1KneeLeft.g(:,3,2),'b--')
for i=1:3
    for j=1:2
IndoorGait1KneeLeftFiltered.g(:,i,j)=lowpass(IndoorGait1KneeLeft.g(:,i,j),2.5,1);
    end
end
plot(IndoorGait1KneeLeftFiltered.g(:,1,1),'r','LineWidth',2)
hold on
plot(IndoorGait1KneeLeftFiltered.g(:,2,1),'g','LineWidth',2)
plot(IndoorGait1KneeLeftFiltered.g(:,3,1),'b','LineWidth',2)
plot(IndoorGait1KneeLeftFiltered.g(:,1,2),'r--','LineWidth',2)
plot(IndoorGait1KneeLeftFiltered.g(:,2,2),'g--','LineWidth',2)
plot(IndoorGait1KneeLeftFiltered.g(:,3,2),'b--','LineWidth',2)

IndoorGait1KneeLeft.a=IndoorGait1KneeLeftFiltered.a;
IndoorGait1KneeLeft.g=IndoorGait1KneeLeftFiltered.g;

% get left knee joint axes wrt sensors
IndoorGait1KneeLeft.j=getaxes(cat(3,IndoorGait1KneeLeft.g(:,:,1),IndoorGait1KneeLeft.g(:,:,2)));


% check axes directions
c=[1;0;0];
j_1=IndoorGait1KneeLeft.j(:,1);
j_2=IndoorGait1KneeLeft.j(:,2);
g_1=IndoorGait1KneeLeft.g(:,:,1);
g_2=IndoorGait1KneeLeft.g(:,:,2);
x_1=cross(j_1,c);
    y_1=cross(j_1,x_1);
    x_2=cross(j_2,c);
    y_2=cross(j_2,x_2);
    
for i=1:400
    vector_1(i,:)=[dot(g_1(i,:),x_1) dot(g_1(i,:),y_1)];
    vector_2(i,:)=[dot(g_2(i,:),x_2) dot(g_2(i,:),y_2)];
    vel1(i)=dot(g_1(i,:),j_1);
    vel2(i)=dot(g_2(i,:),j_2);
end

figure
plot(vel1)
hold on
plot(vel2,'k')
plot(-vel1+vel2,'r')

dt=1/60;
alpha_1(1)=0;
alpha_2(1)=0;
alpha_3(1)=0;
alpha_4(1)=0;
for i=2:min(length(vel1),length(vel2))
    alpha_1(i)=alpha_1(i-1)+(vel1(i)-vel2(i))*dt;
    alpha_2(i)=alpha_2(i-1)+(vel2(i)+vel1(i))*dt;
    alpha_3(i)=alpha_3(i-1)+(vel2(i)-vel1(i))*dt;
    alpha_4(i)=alpha_4(i-1)+(-vel2(i)-vel1(i))*dt;
end
figure
plot(alpha_1,'r')
hold on
plot(alpha_2,'b')
plot(alpha_3,'g')
plot(alpha_4,'k')

figure
plot3([0 j_1(1)],[0 j_1(2)],[0 j_1(3)],'r')
hold on
plot3([0 j_2(1)],[0 j_2(2)],[0 j_2(3)],'b')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal

figure
plot(vector_1(:,1)-vector_1(1,1),vector_1(:,2)-vector_1(1,2),'k-*')
hold on
plot(vector_2(:,1)-vector_2(1,1),vector_2(:,2)-vector_2(1,2),'r-*')



%IndoorGait1KneeLeft.j(:,1)=flipud(IndoorGait1KneeLeft.j(:,1));
%IndoorGait1KneeLeft.j(2,1)=-IndoorGait1KneeLeft.j(2,1);
% left knee joint location
[IndoorGait1KneeLeft.o_1_arbit_left,IndoorGait1KneeLeft.o_2_arbit_left]=getjoints(IndoorGait1KneeLeft.g(:,:,1), IndoorGait1KneeLeft.g(:,:,2), IndoorGait1KneeLeft.a(:,:,1), IndoorGait1KneeLeft.a(:,:,2))

% thigh flexion from Seel
[IndoorGait1KneeLeft.alpha_left_gyr,IndoorGait1KneeLeft.alpha_left_acc,IndoorGait1KneeLeft.alpha_left_acc_filt,IndoorGait1KneeLeft.alpha_left_fus,IndoorGait1KneeLeft.alpha_left_R] = ...
    getflexion(IndoorGait1KneeLeft.j(:,1),IndoorGait1KneeLeft.j(:,2),IndoorGait1KneeLeft.g(:,:,1),IndoorGait1KneeLeft.g(:,:,2),IndoorGait1KneeLeft.a(:,:,1),IndoorGait1KneeLeft.a(:,:,2),IndoorGait1KneeLeft.o_1_arbit_left,IndoorGait1KneeLeft.o_2_arbit_left,IndoorGait1KneeLeft.thigh_left_R,IndoorGait1KneeLeft.shank_left_R)

save JangleLKFLEXalex.mat
load JangleLKFLEXalex.mat
figure
plot(IndoorGait1KneeLeft.alpha_left_acc.*180/pi)
hold on
plot(IndoorGait1KneeLeft.alpha_left_gyr.*180/pi,'r')
plot(IndoorGait1KneeLeft.alpha_left_fus.*180/pi,'g')
plot(IndoorGait1KneeLeft.alpha_left_R.*180/pi,'k')

%IndoorGait1KneeLeft.alpha_left_fus=-(IndoorGait1KneeLeft.alpha_left_fus-mean(IndoorGait1KneeLeft.alpha_left_fus));
%IndoorGait1KneeLeft.alpha_left_fus=IndoorGait1KneeLeft.alpha_left_fus-mean(IndoorGait1KneeLeft.alpha_left_fus(1:20));
%IndoorGait1KneeLeft.alpha_left_fus=pi/2-IndoorGait1KneeLeft.alpha_left_fus;
figure
hold on
plot(IndoorGait1KneeLeft.time,IndoorGait1KneeLeft.alpha_left_fus.*180/pi,'r','LineWidth',1)
legend('Knee Flexion-Extension left','Knee Flexion-Extension right')
xlabel('time [s]')
ylabel('flexion [deg]')
title('Indoor knee flexion/extension')

%Indoor left knee flexion

IndoorGait1LeftKnee.flex{1}=IndoorGait1KneeLeft.alpha_left_fus(191:263);
IndoorGait1LeftKnee.flex{2}=IndoorGait1KneeLeft.alpha_left_fus(263:338);
IndoorGait1LeftKnee.flex{3}=IndoorGait1KneeLeft.alpha_left_fus(338:414);


% normalise time and fit smoothing spline (to get vectors the same length to allow stats comparison

for i=1:3
    IndoorGait1LeftKnee.time{i}=0:1/(length(IndoorGait1LeftKnee.flex{i})-1):1
    fitobject=fit(IndoorGait1LeftKnee.time{i}',IndoorGait1LeftKnee.flex{i}','smoothingspline');
    IndoorGait1LeftKnee.interpFlex{i}=feval(fitobject,0:0.01:1);
end

%%%Standard deviation
IndoorGait1LeftKneeFlexStd=std([IndoorGait1LeftKnee.interpFlex{1}';IndoorGait1LeftKnee.interpFlex{2}';IndoorGait1LeftKnee.interpFlex{3}']);
%%%%Standard error
IndoorGait1LeftKneeFlexSem=IndoorGait1LeftKneeFlexStd/sqrt(length(IndoorGait1LeftKneeFlexStd));
%%%Mean
IndoorGait1LeftKneeFlexMean=mean([IndoorGait1LeftKnee.interpFlex{1}';IndoorGait1LeftKnee.interpFlex{2}';IndoorGait1LeftKnee.interpFlex{3}']);
%%%%CI
IndoorGait1LeftKneeFlex_Upper_limit=IndoorGait1LeftKneeFlexMean+1.66*IndoorGait1LeftKneeFlexStd;
IndoorGait1LeftKneeFlex_Lower_limit=IndoorGait1LeftKneeFlexMean-1.66*IndoorGait1LeftKneeFlexStd;

% plot results
figure
plot(IndoorGait1LeftKnee.time{1},IndoorGait1LeftKnee.flex{1}.*180/pi,'k')
hold on
plot(0:0.01:1,IndoorGait1LeftKnee.interpFlex{1}.*180/pi,'r--')
plot(IndoorGait1LeftKnee.time{2},IndoorGait1LeftKnee.flex{2}.*180/pi,'k')
plot(0:0.01:1,IndoorGait1LeftKnee.interpFlex{2}.*180/pi,'r--')
plot(IndoorGait1LeftKnee.time{3},IndoorGait1LeftKnee.flex{3}.*180/pi,'k')
plot(0:0.01:1,IndoorGait1LeftKnee.interpFlex{3}.*180/pi,'r--')
plot(0:0.01:1,IndoorGait1LeftKneeFlexMean.*180/pi,'r','LineWidth',3)
plot(0:0.01:1,IndoorGait1LeftKneeFlex_Upper_limit.*180/pi,'k--','LineWidth',2)
plot(0:0.01:1,IndoorGait1LeftKneeFlex_Lower_limit.*180/pi,'k--','LineWidth',2)
xlabel('time [s]')
ylabel('Flexion-Extension [deg]')
title('Indoor intra-gait step cycle knee flexion/extension left leg')

IndoorGait1LeftKnee_RMSE1=(rms(IndoorGait1LeftKnee.interpFlex{1}.*180/pi-IndoorGait1LeftKneeFlexMean'.*180/pi));
IndoorGait1LeftKnee_RMSE2=(rms(IndoorGait1LeftKnee.interpFlex{2}.*180/pi-IndoorGait1LeftKneeFlexMean'.*180/pi));
IndoorGait1LeftKnee_RMSE3=(rms(IndoorGait1LeftKnee.interpFlex{3}.*180/pi-IndoorGait1LeftKneeFlexMean'.*180/pi));

%save Gait1_Xsens_Knee_Flex.mat

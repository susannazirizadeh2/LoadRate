clear all

rand('state',0)
randn('state',0)

% acc & gyro data gait1
raw{1}=importdata('par1_5km_100_00341957.txt');
raw{2}=importdata('par1_5km_100_00341958.txt');

nOpt=200;

n=length(raw{1}.data);
time=(raw{1}.data(:,1)-raw{1}.data(1,1)).*(1/60);

a(:,:,1)=raw{1}.data(1201:n-600,7:9);
a(:,:,2)=raw{2}.data(1201:n-600,7:9);

for i=1:3
    for j=1:2
        a(:,i,j)=lowpass(raw{j}.data(1201:n-600,9+i),2.5,1);
    end
end
% 
% figure
% plot(a(:,1,1),'r')
% hold on
% plot(a(:,2,1),'g')
% plot(a(:,3,1),'b')
% plot(a(:,1,2),'r--')
% plot(a(:,2,2),'g--')
% plot(a(:,3,2),'b--')


for i=1:3
    for j=1:2
        g(:,i,j)=lowpass(raw{j}.data(1201:n-600,9+i),2.5,1);
    end
end

% figure
% plot(g(:,1,1),'r')
% hold on
% plot(g(:,2,1),'g')
% plot(g(:,3,1),'b')
% plot(g(:,1,2),'r--')
% plot(g(:,2,2),'g--')
% plot(g(:,3,2),'b--')



% get left knee joint axes wrt sensors
j=getaxes(cat(3,g(1:nOpt,:,1),g(1:nOpt,:,2)));


% check axes directions
c=[1;0;0];
j_1=j(:,1);
j_2=j(:,2);
g_1=g(:,:,1);
g_2=g(:,:,2);
x_1=cross(j_1,c);
    y_1=cross(j_1,x_1);
    x_2=cross(j_2,c);
    y_2=cross(j_2,x_2);
    
% for i=1:400
%     vector_1(i,:)=[dot(g_1(i,:),x_1) dot(g_1(i,:),y_1)];
%     vector_2(i,:)=[dot(g_2(i,:),x_2) dot(g_2(i,:),y_2)];
%     vel1(i)=dot(g_1(i,:),j_1);
%     vel2(i)=dot(g_2(i,:),j_2);
% end

% figure
% plot(vel1)
% hold on
% plot(vel2,'k')
% plot(-vel1+vel2,'r')
% 
% dt=1/60;
% alpha_1(1)=0;
% alpha_2(1)=0;
% alpha_3(1)=0;
% alpha_4(1)=0;
% for i=2:min(length(vel1),length(vel2))
%     alpha_1(i)=alpha_1(i-1)+(vel1(i)-vel2(i))*dt;
%     alpha_2(i)=alpha_2(i-1)+(vel2(i)+vel1(i))*dt;
%     alpha_3(i)=alpha_3(i-1)+(vel2(i)-vel1(i))*dt;
%     alpha_4(i)=alpha_4(i-1)+(-vel2(i)-vel1(i))*dt;
% end
% figure
% plot(alpha_1,'r')
% hold on
% plot(alpha_2,'b')
% plot(alpha_3,'g')
% plot(alpha_4,'k')

figure
plot3([0 j_1(1)],[0 j_1(2)],[0 j_1(3)],'r')
hold on
plot3([0 j_2(1)],[0 j_2(2)],[0 j_2(3)],'b')
title('Axes of the sensor for the right leg for 5kmh 110% Participant1')
legend('Lower leg','Upper leg')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
print -depsc ./kneeaxes_right_5kmh_110_id_P1.eps

% figure
% plot(vector_1(:,1)-vector_1(1,1),vector_1(:,2)-vector_1(1,2),'k-*')
% hold on
% plot(vector_2(:,1)-vector_2(1,1),vector_2(:,2)-vector_2(1,2),'r-*')



%IndoorGait1KneeLeft.j(:,1)=flipud(IndoorGait1KneeLeft.j(:,1));
%IndoorGait1KneeLeft.j(2,1)=-IndoorGait1KneeLeft.j(2,1);
% left knee joint location
[o_1_arbit,o_2_arbit]=getjoints(g(1:nOpt,:,1),g(1:nOpt,:,2),a(1:nOpt,:,1),a(1:nOpt,:,2));

% thigh flexion from Seel
[alpha_gyr,alpha_acc,alpha_acc_filt,alpha_fus] = ...
    getflexion(j(:,1),j(:,2),g(:,:,1),g(:,:,2),a(:,:,1),a(:,:,2),o_1_arbit,o_2_arbit);


% figure
% plot(alpha_acc.*180/pi)
% hold on
% plot(alpha_gyr.*180/pi,'r')
% plot(alpha_fus.*180/pi,'g')


[pks,locs] = findpeaks(alpha_fus)

count=1;
for i=2:2:length(pks)-2
    cycle{count}=alpha_fus(locs(i):locs(i+2));
    count = count+1;
end


figure
hold on
for i=1:length(cycle)
    cycletime{i}=0:1/(length(cycle{i})-1):1;
    fitobject=fit(cycletime{i}',cycle{i}','smoothingspline');
    interpAngle(i,:)=feval(fitobject,0:0.01:1);
    plot([0:0.01:1],interpAngle(i,:).*180/pi,'k--')
end

meanAngle=mean(interpAngle);
plot([0:0.01:1],(meanAngle.*180/pi),'r','LineWidth',2)
title('Knee angle in gait for right leg for 5kmh 110% Participant1')
xlabel('Gait cycle')
ylabel('Flexion Angle [deg]')
print -depsc ./kneeangel_right_5kmh_110_id_P1.eps

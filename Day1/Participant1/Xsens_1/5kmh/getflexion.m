function [alpha_gyr,alpha_acc,alpha_acc_filt,alpha_fus]=getflexion(j_1,j_2,g_1,g_2,a_1,a_2,o_1_arbit,o_2_arbit)

% flexion from Seel
c=[1;0;0];


    % using accelerometers
    [gamma_1,gamma_2] = gamma_calc(o_1_arbit,o_2_arbit,g_1,g_2);
    x_1=cross(j_1,c);
    y_1=cross(j_1,x_1);
    x_2=cross(j_2,c);
    y_2=cross(j_2,x_2);
    for i=1:length(gamma_1)
        a1_tilde=a_1(i,:)-gamma_1(i);
        a2_tilde=a_2(i,:)-gamma_2(i);
        vector_1=[dot(a1_tilde,x_1) dot(a1_tilde,y_1)];
        vector_2=[dot(a2_tilde,x_2) dot(a2_tilde,y_2)];
        alpha_acc(i)=acos(dot(vector_1/norm(vector_1),vector_2/norm(vector_2)));
    end
    
    % filter alpha_acc
    alpha_acc_filt=lowpass(alpha_acc,0.1,1/60);
    
    % using gyros
dt=1/60;
alpha_gyr(1)=alpha_acc(1);
for i=2:min(length(g_1),length(g_2))
    alpha_gyr(i)=alpha_gyr(i-1)+(dot(g_2(i,:),j_2)-dot(g_1(i,:),j_1))*dt;
end
    
    %fusing gyro and accelerometer angles
    alpha_fus(1)=alpha_acc(1);
    lambda=0.02;
    for i=2:length(alpha_acc)
        alpha_fus(i)=lambda*alpha_acc_filt(i)+(1-lambda)*(alpha_fus(i-1)+(alpha_gyr(i)-alpha_gyr(i-1)));
    end
    


% if nargin==10
%     % using rotation matrices
%     for i=1:min(length(R_1),length(R_2))
%         flex_1=R_1(:,:,i)*cross(j_1,c);
%         flex_2=R_2(:,:,i)*cross(j_2,c);
%         alpha_R(i)=acos(dot(flex_1/norm(flex_1),flex_2/norm(flex_2)));
%     end
% end
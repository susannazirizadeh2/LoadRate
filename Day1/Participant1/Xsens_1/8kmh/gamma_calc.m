function [gamma_1,gamma_2] = gamma_calc(o_1,o_2,g_1, g_2)


n=length(g_1);

% time derivates of angular rate 
gDot_1=zeros(n,3);
gDot_2=zeros(n,3);
gamma_1=zeros(n,3);
gamma_2=zeros(n,3);
for t=3:n-2
    %tDelta=(time(t+2)-time(t-2))/4;
    tDelta=1/60;
    gDot_1(t,:) = (g_1(t-2,:) - 8.*g_1(t-1,:) + 8.*g_1(t+1,:) - g_1(t+2,:))/(12*tDelta);
    gDot_2(t,:) = (g_2(t-2,:) - 8.*g_2(t-1,:) + 8.*g_2(t+1,:) - g_2(t+2,:))/(12*tDelta);
    gamma_1(t,:) = cross(g_1(t,:), cross( g_1(t,:), o_1)) + cross(gDot_1(t,:), o_1);
    gamma_2(t,:) = cross(g_2(t,:), cross( g_2(t,:), o_2)) + cross(gDot_2(t,:), o_2);
end


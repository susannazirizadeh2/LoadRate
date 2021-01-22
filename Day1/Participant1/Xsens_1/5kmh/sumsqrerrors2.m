function ssqe =sumsqrerrors2(x, g_1, g_2, a_1, a_2)
o_1=x(1:3);
o_2=x(4:6);

n=length(g_1);

% time derivates of angular rate 
gDot_1=zeros(n,3);
gDot_2=zeros(n,3);
radtan_a1=zeros(n,3);
radtan_a2=zeros(n,3);
for t=3:n-2
    %tDelta=(time(t+2)-time(t-2))/4;
    tDelta=1/60;
    gDot_1(t,:) = (g_1(t-2,:) - 8.*g_1(t-1,:) + 8.*g_1(t+1,:) - g_1(t+2,:))/(12*tDelta);
    gDot_2(t,:) = (g_2(t-2,:) - 8.*g_2(t-1,:) + 8.*g_2(t+1,:) - g_2(t+2,:))/(12*tDelta);
    radtan_a1(t,:) = cross (g_1(t,:), cross( g_1(t,:), o_1)) + cross (gDot_1(t,:), o_1);
    radtan_a2(t,:) = cross (g_2(t,:), cross( g_2(t,:), o_2)) + cross (gDot_2(t,:), o_2);
end

e=zeros(n,1);
for i=1:n
    e(i)= norm(a_1(i,:)-radtan_a1(i,:))-norm(a_2(i,:)-radtan_a2(i,:));
end

ssqe =sum(e.^2);


function ssqe =sumsqrerrors(x,g)

[n,~,k]=size(g);
phi=x(1:k);
theta=x(k+1:k*2);

for i=1:k
    j(:,i)=[cos(phi(i))*cos(theta(i)) cos(phi(i))*sin(theta(i)) sin(phi(i))]';
end

e=zeros(n,1);
term=zeros(n,k);
for i=1:n
    for ii=1:k
        term(i,ii)=norm(cross(g(i,:,ii),j(:,ii)));
    end
    e(i)=term(i,1);
    for ii=2:k
        e(i)=e(i)-term(i,ii);
    end
end
ssqe=sum(e.^2);
% for ii=1:k
%        e(:,ii)=mean(term,2)-term(:,ii);
% end
% ssqe=sum(sum(e.^2));


function j=getaxes(g)

[~,~,k]=size(g);
options=gaoptimset('Display','iter','PopulationSize',50,'Generations',20);
[bestAngles,minSumSqrError]=ga(@(x)sumsqrerrors(x,g),k*2,[],[],[],[],ones(1,k*2).*-1,ones(1,k*2),[],options)
options=optimset('Display','iter');
[bestAngles,minSumSqrError]=fmincon(@(x)sumsqrerrors(x,g),bestAngles,[],[],[],[],[],[],[],options)
phi=bestAngles(1:k);
theta=bestAngles(k+1:k*2);
for i=1:k
    j(:,i)=[cos(phi(i))*cos(theta(i)) cos(phi(i))*sin(theta(i)) sin(phi(i))]';
end
function [o_1_arbit,o_2_arbit]=getjoints(g_1, g_2, a_1, a_2)

options=gaoptimset('Display','iter','PopulationSize',50,'Generations',20);
[bestLocation,minSumSqrError]=ga(@(x)sumsqrerrors2(x, g_1, g_2, a_1, a_2),6,[],[],[],[],[0 0 0 0 0 0],[1 1 1 1 1 1],[],options)
options=optimset('Display','iter');
[bestLocation,minSumSqrError]=fmincon(@(x)sumsqrerrors2(x, g_1, g_2, a_1, a_2),bestLocation,[],[],[],[],[],[],[],options)
o_1_arbit=bestLocation(1:3);
o_2_arbit=bestLocation(4:6);
%% calc rigitdy matrix
% file: calcRigitdyMatrix.m
% author: Ido Sherf 
% date: 01/02/2024
% description: calculate the rigitdy matrix for given los table 
function R = calcRigitdyMatrix(los_table,agents_list)
p = 2; % we work in R^p
n = size(los_table,1); % there is |E| row in R
m = length(agents_list)*p; % there is p*|V| cols in R
R = zeros(n,m);

for  ie = 1:n
    e = los_table(ie,:);
    v1 = e(1);
    v2 = e(2);
    p1 = e(5:5+p-1);
    p2 = e(5+p:5+2*p-1);
    p1p2 = p1-p2;
    p2p1 = -p1p2;
    v1_r_idx = (v1-1)*2 + 1 : (v1-1)*2 + p;
    v2_r_idx = (v2-1)*2 + 1 : (v2-1)*2 + p;
    R(ie,v1_r_idx) = p1p2;
    R(ie,v2_r_idx) = p2p1;
end


function J = cost_function_trilateration(X,D,P)

% n meas
nmeas = numel(D);

% get cost function
Pdiff = P - X;

% init
J = 0;

% build diff
for i=1:nmeas
    J = J + (Pdiff(i,:)*Pdiff(i,:)' - D(i)^2)^2;
end

end
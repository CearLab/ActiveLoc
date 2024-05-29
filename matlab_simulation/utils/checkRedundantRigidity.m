% check redundant rigidity
function [isRedundant, improveList] = checkRedundantRigidity(losTAB,agents_list)

% remove one edge at a time and check rigidity
Nedges = size(losTAB,1);

% init
isRedundant = 0;
fail = 0;
improveList = [];

% cycle over edges
for i=1:Nedges

    % get LOS and check rigidity
    losTMP = losTAB;
    losTMP(i,:) = [];
    isRigid_ = isRigid(losTMP,agents_list);

    % if not rigid, we're alrready done
    if ~isRigid_
        fail = 1;
        improveList(end+1,1) = i;
    end
end

% huray it's redundant
if ~fail
    isRedundant = 1;
end

end
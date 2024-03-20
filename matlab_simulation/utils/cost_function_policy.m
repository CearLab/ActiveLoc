% cost function
function J = cost_function_policy(x,CASE,IDloc)

% get manager and team
manager = AgentManager.getInstance();

% get plane or space info
p = manager.WS.p;

% CASE 1
% I have neighbors but less than two of them are localized.
% I just move away from them.
if CASE == 1

    % stay here
    J = 0;
    return;

    % get agent list only of those localized
    agents_list = [obj.neigh.ID(IDloc,1), obj.neigh.Pest(IDloc,:)];

    % define pdes as the barycenter of them
    G = mean(agents_list(find(agents_list(:,end) == 1),2:3),1);

    % compute your distance from the barycenter
    delta = G - x;
    dbar = norm(delta);

    % compute bearing from initial condition
    currbear = atan2(delta(2),delta(1));
    bear = abs(currbear - pi*sign(currbear));

    % maximize this distance
    J = 0/dbar + 1*bear;

    % CASE 2
    % I have at least three neighbors localized.
    % I want to move to maximize rigidity.
elseif CASE == 2

    % stay here
    J =0;
    return;

    % get agent list only of those localized
    agents_list = [obj.neigh.ID(IDloc,1), obj.neigh.Pest(IDloc,:)];
    agents_list(end+1,:) = [obj.agent_number x 0];

    % get LOS tab only for localized nodes and itself
    losTmp = obj.neigh.LOStab(find(obj.neigh.LOStab(:,end) == 1), :);
    rowLocStart = [];
    rowLocEnd = [];
    for j=1:size(agents_list,1)
        rowLocStart = [rowLocStart; find(losTmp(:,3) == agents_list(j,1))];
        rowLocEnd = [rowLocEnd; find(losTmp(:,4) == agents_list(j,1))];
    end
    rowLoc = intersect(rowLocStart,rowLocEnd);
    losTAB = losTmp(rowLoc,:);

    % replace the estimated location with the actual x
    for i=1:size(losTAB,1)
        % if agent is the first in the row pair
        if losTAB(i,3) == obj.agent_number
            losTAB(i,5:5+p-1) = x;
            losTAB(i,5+2*p:5+3*p-1) = x;
        end
        % if agent is the second in the row pair
        if losTAB(i,4) == obj.agent_number
            losTAB(i,5+p:5+2*p-1) = x;
            losTAB(i,5+3*p:5+4*p-1) = x;
        end
    end

    % check theorem
    [kconn, improveListConn] = checkKconnectivity(obj,losTAB,agents_list,3);
    [isRedundant, improveListRed] = checkRedundantRigidity(obj,losTAB,agents_list);

    % if rigid, get lambda4 and maximize
    if kconn && isRedundant
        [isRigid, e] = obj.isRigid(losTAB,agents_list);
        J = 1/min(e);
        % if not globally rigid not good
    else
        J = 1e5;
    end

end

end
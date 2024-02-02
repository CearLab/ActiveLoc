%% draw los map
% file: drawLosMap.m
% author: Ido Sherf 
% date: 01/02/2024
% description: plot the los table on map
function drawLosMap(los_table,varargin)

% if no figure handle provided call map
if isempty(varargin)
    Map.setFigure();
else
    figure(varargin{1});
    if numel(varargin) == 2
        color = getColorByNumber(varargin{2});
    end    
end

% get number of pairs and get Ni (rows)
n = size(los_table);
n = n(1);

% for each Ai draw LOS
for i = 1:n

    % get locs
    c_line = los_table(i,:);

    % get origin and endpoint in XoY
    x1 = c_line(5);
    x2 = c_line(7);
    y1 = c_line(6);
    y2 = c_line(8);

    % plot
    plot([x1,x2],[y1,y2],'Color',color,'LineStyle','--','LineWidth', 1);
end


end


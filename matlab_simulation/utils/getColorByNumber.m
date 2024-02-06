%% Get color by number
% file: getColorByNumber.m
% author: Ido Sherf 
% date: 22/01/2024
% description: hash table for colors
function color = getColorByNumber(num)

    % define hast table
    colors = {
        
        [1, 0, 0],    % Red             1     
        [0, 0, 1],    % Blue            2
        [0, 1, 1],    % Cyan            3
        [1, 0, 1],    % Magenta         4
        [1, 1, 0],    % Yellow          5
        [0, 0, 0],    % Black           6
        [1, 1, 1],     % White          7
        [0, 1, 0],    % Green           8
        [0.6, 0, 0],    % Dark green    9
        
        % Add more custom colors here if needed
    };

    % get color by number
    color = colors{num};

end
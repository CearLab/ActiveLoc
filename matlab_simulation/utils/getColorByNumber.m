%% Get color by number
% file: getColorByNumber.m
% author: Ido Sherf 
% date: 22/01/2024
% description: hash table for colors
function color = getColorByNumber(num)

    % define hast table
    colors = {
        
        [1, 0, 0],    % Red        
        [0, 0, 1],    % Blue
        [0, 1, 1],    % Cyan
        [1, 0, 1],    % Magenta
        [1, 1, 0],    % Yellow
        [0, 0, 0],    % Black
        [1, 1, 1],     % White
        [0, 1, 0],    % Green
        
        % Add more custom colors here if needed
    };

    % get color by number
    color = colors{num};

end
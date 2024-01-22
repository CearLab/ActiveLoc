function color = get_color_by_number(num)
colors = {
    [1, 0, 0],    % Red
    [0, 1, 0],    % Green
    [0, 0, 1],    % Blue
    [0, 1, 1],    % Cyan
    [1, 0, 1],    % Magenta
    [1, 1, 0],    % Yellow
    [0, 0, 0],    % Black
    [1, 1, 1]     % White
    % Add more custom colors here if needed
};
color = colors{num};
end
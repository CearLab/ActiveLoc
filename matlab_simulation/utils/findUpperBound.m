function p = findUpperBound(x, y)
% Initial fit
p = polyfit(x, y, 1); % Fit a line (1st degree polynomial)

% Initialize loop variables
maxIterations = 5; % Maximum number of iterations to prevent infinite loops
iteration = 0;
removedPoints = true;

while iteration < maxIterations && removedPoints
    % Calculate the y values from the current fit
    yFit = polyval(p, x);
    
    % Find points above the fitted line
    BelowLine = y < yFit;
    
    if any(BelowLine)
        % There are points above the line; remove them
        x(BelowLine) = [];
        y(BelowLine) = [];
        
        % Refit the line
        p = polyfit(x, y, 1);
        
        % Mark that we've removed points and need to possibly iterate again
        removedPoints = true;
    else
        % No points are above the line; we're done
        removedPoints = false;
    end
    
    iteration = iteration + 1;
end
end
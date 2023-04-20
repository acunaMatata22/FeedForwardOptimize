function accuracy = Peval(paramFun, paramType, numTests, stepSizes)
% based on paramType, set the parameters using the function provided and a
% random sample of tests to evaluate accuracy. Tests for steps of sizes
% bewteen minimum and maximum for the controller

% NOTE: ask how to select minimum resolution baseed on controller params
pd = makedist('Loguniform', 'Lower', -3.5, 'Upper', 1.2);
steps = random(pd,numTests,1);

% Testing loop
for i = 1:numTests
    step = steps(i);
    

% Update parameters
switch paramType
    case 'A'
    % Only Acceleration is specified
    case 'PVAJ'
        % Position, velocity, acceleration, jerk parameters
        pass;
end

end

end
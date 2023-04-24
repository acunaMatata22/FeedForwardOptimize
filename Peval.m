function accuracy = Peval(CTLR, paramFun, paramType, numTests, stepLimits)
% based on paramType, set the parameters using the function provided and a
% random sample of tests to evaluate accuracy. Tests for steps of sizes
% bewteen minimum and maximum for the controller

% NOTE: ask how to select minimum resolution baseed on controller params
pd = makedist('Loguniform', 'Lower', stepLimits(1), 'Upper', stepLimits(2));
steps = random(pd,numTests,1);
pd2 = makedist('Loguniform', 'Lower', 1E-3, 'Upper', 2);
randShift = random(pd2,numTests,1);
errorHist = zeros(numTests,1);

% Setup
% CTLR = controller([0],300);
epsilon_settled = .1;

% Testing loop
for i = 1:numTests
    if mod(i,100) == 0 % Home every 20 moves to redistribute lubricant
        CTLR.Home;
    end
    c_step = steps(i);
    newGains = feval(paramFun,c_step);
%     fprintf('Step: %f, Gain: %f\n', c_step, newGains);

    % Goto -1
    CTLR.Move(-2);

    % Random move to establish backlash alternating forward/backwards
    if mod(i,2) == 0
        CTLR.Move(randShift(i));
    else
        CTLR.Move(-randShift(i));
    end

    % Test move
    CTLR = CTLR.GetDataCol(c_step);
    CTLR.UpdateParam(paramType, newGains);
    stepError = CTLR.MeasureStep(c_step);
    errorHist(i) = stepError;
end

accuracy = sum(errorHist)/numTests;
end
%% CONSTANTS
GAINAFF = A3200ParameterId.GainAff;
GAINPFF = A3200ParameterId.GainPff;
GAINVff = A3200ParameterId.GainVff;
GAINJff = A3200ParameterId.GainJff;

%% OPTIMIZATION PARAMS
% Aff = 66.04; % Initialize acceleration gain
speed = 300;
step_sizes = logspace(-3.5,1.2,10);
epsilon_settled = .1;
epsilon_converged = .3;
delta = 1;
batch = 8; % Number of times repeating a value
alpha = 4; % Learning rate
beta = 100000;

paramType = 'A';
paramsList = [GAINAFF];
curGains = [50]; % Starting values

%% SETUP CONTROLLER
CTLR = Controller([0],300);

%% SETUP
samplePeriod = 1; % in ms
n_steps = length(step_sizes);
n_params = length(paramsList);
% ErrorData = zeros(n_vals*n_steps,3); 
OptParams = zeros(n_steps,n_params);
Gradients = zeros(1,n_params);

%% TEST LOOP

% Iterate over step sizes
for i = 1:n_steps
%     CTLR.Home; % Home between step sizes to redistribute lube?   
    step = step_sizes(i);    % Change number of samples
    CTLR.GetDataCol(step);
    prevGains = zeros(1,n_params);
    epochs = 1;

% Gradient Descent
while rmse(curGains, prevGains) > epsilon_converged 
    CTLR.UpdateParam(paramType, curGains);
%     baselineError = CTLR.multiTest(step, repeat)

    % Iterate over the parameters to find the gradients
    for param = 1:n_params
        % central difference approx.
        dplusGains = curGains;
        dminusGains = curGains;
        dplusGains(param) = curGains(param) + delta;
        dminusGains(param) = curGains(param) - delta;

        CTLR.UpdateParam(paramType, dplusGains);
        plusError = CTLR.multiTest(step, batch);
        CTLR.UpdateParam(paramType, dminusGains);
        minusError = CTLR.multiTest(step,batch);
        
        Gradients(param) = (plusError - minusError) / 2*delta;
    end
    
    % Update Params based on learning rate
    prevGains = curGains;
    learning = Gradients*(alpha*beta)/epochs^.7;
    curGains = curGains - learning
    epochs = epochs+1;
end

% TODO
OptParams(i,:) = curGains;
disp(['Optimum Parameter for step ', num2str(step), ': ', num2str(OptParams(i))]);

end

%% Plot Optimization Data

f3 = figure(2);
semilogx(step_sizes, OptParams);
title('Optimum Gains vs. Step Size');
ylabel('Optimum Aff Gain');
xlabel('Step Size (mm)');

%% Create Fit
logSizes = log(step_sizes);
% TODO

%% Save Data

% Save Data
% Data = table(ErrorData(:,1), ErrorData(:,2), ErrorData(:,3), 'VariableNames', {'Step', 'Gain', 'Step_Error'});
% writematrix(ErrorData,'Results/DataSamples.csv');
writematrix(OptParams,'Results/OptParams.csv');
save('Results/DataSamples.mat', 'Data');
save('Results/OptimumGains.mat', 'OptParams');
saveas(f3,'Results/Optimization Curve.png');


% Free Controller Handles
disp('SUCCESS');
CTLR.Free;
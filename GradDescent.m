%% OPTIMIZATION PARAMS
speed = 300;
step_sizes = logspace(-3.5,1.7,6)';
epsilon_settled = .1;
epsilon_converged = .3;
delta = [.3 .3 .9 .3];
batch = 6; % Number of times repeating a value
alpha = [3E5 3E5 12E5 4E5]; % Learning rate

paramType = 'PVAJ';
curGains = [1 6 50 5]; % Starting values

%% SETUP CONTROLLER
CTLR = Controller([0],300);

%% SETUP
samplePeriod = 1; % in ms
n_steps = length(step_sizes);
n_params = length(curGains);
% ErrorData = zeros(n_vals*n_steps,3); 
OptParams = zeros(n_steps,n_params);
Gradients = zeros(1,n_params);

%% TEST LOOP
% 
% close all
% f1 = figure(1);
% f1.Position = [200 200 900 600];

% Iterate over step sizes
for i = 1:n_steps
    h = animatedline; % New line for a new step size
    h.Color = [mod(step*43.419,1) mod(step*63.713,1) mod((step*29.301),1)];

%     CTLR.Home; % Home between step sizes to redistribute lube?   
    step = step_sizes(i);    % Change number of samples
    CTLR.GetDataCol(step);
    prevGains = zeros(1,n_params);
    epoch = 1;
    gradHist = [];

    % -- Adam Hyperparameters --
    m = 0;
    v = 0;
    beta1 = .9;
    beta2 = .999;
    eps = 1E-8;

    % -- Momentum hyperparamters --
    gamma = .65; % Relative importance of previous learning rate
    learning = zeros(1,n_params);

% Gradient Descent (use rmse for when gains are on constraint
while rmse(curGains,prevGains) > epsilon_converged || batch < 10 || epoch < 3 && epoch < 20
    CTLR.UpdateParam(paramType, curGains);

    % NAG lookahead
    predGains = curGains - gamma*learning;

    % -- Varying batch size to help with convergence
    if epoch == 1 % Initial batch size
        batch = 8;
    else
        batch = min([22 max([4 2*round(4/norm(beta.*Gradients)*(epoch+8)^.6)])]);
    end

    % Iterate over the parameters to find the gradients
    for param = 1:n_params
%         % central difference approx. of current gains
%         dplusGains = curGains;
%         dminusGains = curGains;
%         dplusGains(param) = curGains(param) + delta(param);
%         dminusGains(param) = max([0 curGains(param) - delta(param)]);

        % Lookahead Gradient
        dplusGains = predGains;
        dminusGains = predGains;
        dplusGains(param) = predGains(param) + delta(param);
        dminusGains(param) = predGains(param) - delta(param);

        CTLR.UpdateParam(paramType, dplusGains);
        plusError = CTLR.multiTest(step, batch);
        CTLR.UpdateParam(paramType, dminusGains);
        minusError = CTLR.multiTest(step,batch);
        
        Gradients(param) = (plusError - minusError) / 2*delta(param);
    end

    % -- Adam optimizer calcs
%     m = beta1*m + (1-beta1)*Gradients;
%     v = beta2*v + (1-beta2)*Gradients.^2;
%     mhat = m/(1-beta1^epoch);
%     vhat = v/(1-beta2^epoch);
    
    % Calculate the learning rate
    prevGains = curGains;
    % -- Vanilla SGD
    % learning = Gradients.*alpha/epoch^.4;
    % -- Adam
    % learning = alpha.*(mhat/sqrt(vhat+eps));
    % -- Momentum/NAG

    % Dynamically update learning rate
    if epoch > 3
        V = norm(std(alpha.*gradHist(end-2:end,:),0,1));
        beta = alpha / (.3*V);
    else
        beta = alpha;
    end

    learning = (gamma*learning + beta.*Gradients)/(epoch+4)^.55;
    gradHist = [gradHist; Gradients];

    curGains = max([(curGains - learning); .1*ones(1,n_params)]);
% 
%         % Plot
%     figure(f1);
%     addpoints(h,curGains(1),curGains(2));
%     drawnow

        % Display current state
    baselineError = CTLR.multiTest(step, batch);
    disp(['    New Parameters: ', num2str(curGains), ',   Batch Size: ', num2str(batch), ',  Current Error: ', num2str(baselineError)]);

    epoch = epoch+1;
end

% TODO
OptParams(i,:) = curGains;
disp(['-- Optimum Parameters for step ', num2str(step), ': ', num2str(OptParams(i,:))]);
disp(['-- epochs: ', num2str(epoch)]);

end

%% Plot Optimization Data

f3 = figure(2);
stepData = step_sizes*ones(1,n_params);
semilogx(stepData, OptParams);
title('Optimum Gains vs. Step Size');
ylabel('Optimum Gain Value');
xlabel('Step Size (mm)');

%% Create Fit Array
logSizes = log10(step_sizes);
p = cell(n_steps,1);
f = cell(n_steps,1);
for i = 1:n_params
    p{i} = polyfit(logSizes, OptParams(:,i), 3);
    f{i} = fit(logSizes, OptParams(:,i), 'Fourier2');
end

%% Save Data

% Save Data
% Data = table(ErrorData(:,1), ErrorData(:,2), ErrorData(:,3), 'VariableNames', {'Step', 'Gain', 'Step_Error'});
% writematrix(ErrorData,'Results/DataSamples.csv');
Results = table(step_sizes, OptParams);
writetable(Results, 'Results/OptData.csv');
save('Results/OptimumGains.mat', 'OptParams');
saveas(f3,'Results/Optimization Curve.png');
t = datetime('Now');
save(['Fits/PolyArray' char(t)], 'p');
save(['Fits/FourierArray' char(t)], 'f');


% Free Controller Handles
disp('SUCCESS');
CTLR.Free;
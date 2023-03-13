%% SETUP CONTROLLER
disp('Adding the controller Matlab library to the path.')
run('AEROPATH.m')
clear

% Setup parameters
samplePeriod = 1;
dt = samplePeriod*.001;
sampleCount = 200;
axes = [0]; % X (using default names)
GAINAFF = A3200ParameterId.GainAff;

% Performing connections and initialization
handle = A3200Init();
dataCollHandle = DataColInit(handle, samplePeriod, sampleCount);

% Home
disp('Enabling and homing X axis.')
A3200MotionEnable(handle, 1, axes)
A3200MotionHome(handle, 1, axes)

%% SETUP OPTIMIZATION
Aff = 66.04; % Initialize acceleration gain
step_sizes = [.2];
speed = 100;
testValues = 20:4:80;
ErrorHist = zeros(size(testValues));
epsilon = .01;

%% TEST LOOP

iteration = 0;
figure(10);
h = animatedline;
% axis([0 max(testValues) 0 5]);

for iter = 1:length(testValues)
    gain = testValues(iter);

    % Update Parameter
    A3200ParameterSetValue(handle,GAINAFF,0,gain);
    disp(['Testing Aff = ', num2str(A3200ParameterGetValue(handle,GAINAFF,0))]);
    
    % Test of series of Steps
    totalError = 0;
    for i = 1:length(step_sizes)
        step = step_sizes(i);
        step_time = step / speed; % The time before starting error collection
        start_sample = round(step_time / dt);

        % Run step
%         disp('Start collecting the data.')
        A3200DataCollectionStart(handle, dataCollHandle)
%         disp('Execute a step')
        A3200MotionMoveAbs(handle, 1, axes, step, speed)
        A3200MotionWaitForMotionDone(handle, axes, A3200WaitOption.InPosition, -1);
        
        % Collect Data
%         disp('Retrieving all data samples.')
        collectedData = A3200DataCollectionDataRetrieve(handle, dataCollHandle, sampleCount);
        posData = Converter(collectedData(1,:));
        velData = Converter(collectedData(2,:));
        refData = Converter(collectedData(3,:));
        settleInd = abs(refData - step) < epsilon;

        % Return to 0 and compile error
        A3200MotionMoveAbs(handle, 1, axes, 0, speed)
        stepError = posData(settleInd) - refData(settleInd);
        disp(stepError);
        totalError = totalError + rms(stepError);
%         disp(totalError);
        A3200MotionWaitForMotionDone(handle, axes, A3200WaitOption.InPosition, -1);
    end
    disp(['Error = ', num2str(totalError)]);
    ErrorHist(iter) = totalError;
    addpoints(h,gain,totalError);
    drawnow

end
hold off;
% disp('Printing out the collected data.')
% sampleNumber = 1;
% while sampleNumber <= sampleCount
% 	disp(['Position Feedback : ', num2str(posData(sampleNumber))])
% 	disp(['Velocity Feedback : ', num2str(velData(sampleNumber))])
% 	sampleNumber = sampleNumber + 1;
% end

%% Plot Results
figure(2)
subplot(2,1,1)
n = length(posData);
plot(1:n,posData,1:n,refData);
xlabel("Time (ms)");
ylabel("Position (mm)");
subplot(2,1,2)
plot(velData);
xlabel("Time (ms)");
ylabel("Velocity (mm/s)");

%% Free Resources
disp('Freeing the resources used by the data collection configuration.')
A3200DataCollectionConfigFree(dataCollHandle);
disp('Disconnecting from the A3200')
A3200Disconnect(handle);
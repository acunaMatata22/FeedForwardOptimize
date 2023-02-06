%% SETUP CONTROLLER
disp('Adding the controller Matlab library to the path.')
run('AEROPATH.m')

% Setup parameters
samplePeriod = 1;
sampleCount = 200;
axes = [0]; % X (using default names)

% Performing connections and initialization
handle = A3200Init();
dataCollHandle = DataColInit(handle, samplePeriod, sampleCount);

% Home
disp('Enabling and homing X axis.')
A3200MotionEnable(handle, 1, axes)
A3200MotionHome(handle, 1, axes)

%% SETUP OPTIMIZATION
Aff = 66.04; % Initialize acceleration gain
d


%% TEST LOOP

iteration = 0;

while abs(derror) > epsilon_finish
    
    
    for i = 1:length(step_sizes)
        step = step_sizes(i);
        step_time = step / speed; % The time before starting error collection

        % Run step
        disp('Start collecting the data.')
        A3200DataCollectionStart(handle, dataCollHandle)
        disp('Execute a step')
        A3200MotionMoveAbs(handle, 1, axes, step, speed)
        A3200MotionWaitForMotionDone(handle, axes, A3200WaitOption.InPosition, -1);
        
        
        disp('Retrieving all 1000 data samples.')
        collectedData = A3200DataCollectionDataRetrieve(handle, dataCollHandle, 1000);
        
        posData = Converter(collectedData(1,:));
        velData = Converter(collectedData(2,:));
    end


end

% disp('Printing out the collected data.')
% sampleNumber = 1;
% while sampleNumber <= sampleCount
% 	disp(['Position Feedback : ', num2str(posData(sampleNumber))])
% 	disp(['Velocity Feedback : ', num2str(velData(sampleNumber))])
% 	sampleNumber = sampleNumber + 1;
% end

%% Plot Results
figure(1)
subplot(2,1,1)
plot(posData);
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
A3200Disconnect(handle);;
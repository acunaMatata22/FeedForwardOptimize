disp('Adding the controller Matlab library to the path.')
% Add the controller Matlab library to the path
run('AEROPATH.m')

disp('Connecting to the A3200')
handle = A3200Connect;

disp('Creating a data collection configuration.')
dataCollHandle = A3200DataCollectionConfigCreate(handle);

disp('Configuring to collect position feedback and velocity feedback data on the 0th axis.')
A3200DataCollectionConfigAddSignal(dataCollHandle, A3200DataSignal.ProgramPositionFeedback, 0, 0)
A3200DataCollectionConfigAddSignal(dataCollHandle, A3200DataSignal.VelocityFeedback, 0, 0)

disp('Configuring to collect one sample every one millisecond.')
A3200DataCollectionConfigSetPeriod(dataCollHandle, 1)

disp('Configuring to collect 1000 samples.')
A3200DataCollectionConfigSetSamples(dataCollHandle, 1000)

disp('Start collecting the data.')
A3200DataCollectionStart(handle, dataCollHandle)

% Run step


disp('Retrieving all 1000 data samples.')
collectedData = A3200DataCollectionDataRetrieve(handle, dataCollHandle, 1000);

disp('Printing out the collected data.')
sampleNumber = 1;
while sampleNumber <= 1000
	disp(['Position Feedback : ', num2str(collectedData(1, sampleNumber))])
	disp(['Velocity Feedback : ', num2str(collectedData(2, sampleNumber))])
	sampleNumber = sampleNumber + 1;
end

% Encoder counts per primary unit
CountsPerUnit = 3276800;

avgX = sum(collectedData(1,:)) / 1000 / CountsPerUnit


disp('Freeing the resources used by the data collection configuration.')
A3200DataCollectionConfigFree(dataCollHandle);

disp('Disconnecting from the A3200')
A3200Disconnect(handle);
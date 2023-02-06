function dataCollHandle = DataColInit(A3200Handle, samplePeriod, sampleCount)
% Initialize the data collection, configure the connection, and return a handle
disp('Creating a data collection configuration.')
dataCollHandle = A3200DataCollectionConfigCreate(A3200Handle);

disp('Configuring to collect position feedback and velocity feedback data on the 0th axis.')
A3200DataCollectionConfigAddSignal(dataCollHandle, A3200DataSignal.ProgramPositionFeedback, 0, 0)
A3200DataCollectionConfigAddSignal(dataCollHandle, A3200DataSignal.VelocityFeedback, 0, 0)

disp('Configuring to collect one sample every one millisecond.')
A3200DataCollectionConfigSetPeriod(dataCollHandle, samplePeriod)

disp('Configuring to collect 1000 samples.')
A3200DataCollectionConfigSetSamples(dataCollHandle, sampleCount)
end
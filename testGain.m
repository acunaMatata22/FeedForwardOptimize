function [posData,velData,refData] = testGain(handle, dataColHandle,sampleCount, axes, step, speed, gain)
% Perform a step test at a specific gain and return data

GAINAFF = A3200ParameterId.GainAff;

% Update Parameter
A3200ParameterSetValue(handle,GAINAFF,0,gain);
%         disp(['Testing Aff = ', num2str(A3200ParameterGetValue(handle,GAINAFF,0))]);

% Run step
A3200DataCollectionStart(handle, dataColHandle);
A3200MotionMoveAbs(handle, 1, axes, step, speed); % Try Rapid command
A3200MotionWaitForMotionDone(handle, axes, A3200WaitOption.InPosition, -1);

% Collect Data
collectedData = A3200DataCollectionDataRetrieve(handle, dataColHandle, sampleCount);
posData = Converter(collectedData(1,:));
velData = Converter(collectedData(2,:));
refData = Converter(collectedData(3,:));

% Return to 0
A3200MotionMoveAbs(handle, 1, axes, 0, speed)
A3200MotionWaitForMotionDone(handle, axes, A3200WaitOption.InPosition, -1);
end
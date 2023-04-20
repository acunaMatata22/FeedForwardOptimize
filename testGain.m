function [posData,velData,refData] = testGain(Controller, step, speed)
% Perform a step test at a specific gain and return collected data. Returns
% to 0 and waits until reaches 0.

CTLR = Controller;


%         disp(['Testing Aff = ', num2str(A3200ParameterGetValue(handle,GAINAFF,0))]);

% Run step
A3200DataCollectionStart(CTLR.handle, CTLR.dataColHandle);
A3200MotionMoveAbs(CTLR.handle, 1, CTLR.axes, step, speed); % Try Rapid command
A3200MotionWaitForMotionDone(CTLR.handle, CTLR.axes, A3200WaitOption.InPosition, -1);

% Collect Data
collectedData = A3200DataCollectionDataRetrieve(CTLR.handle, CTLR.dataColHandle, CTLR.sampleCount);
posData = Converter(collectedData(1,:));
velData = Converter(collectedData(2,:));
refData = Converter(collectedData(3,:));

% Return to 0
A3200MotionMoveAbs(CTLR.handle, 1, CTLR.axes, 0, speed)
A3200MotionWaitForMotionDone(CTLR.handle, CTLR.axes, A3200WaitOption.InPosition, -1);
end
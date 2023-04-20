classdef Controller
    %CONTROLLER Contains parameters and useful methods for setting up and
    %interfacing with the controller or passing handles.

    %   Detailed explanation goes here
    
    properties
        handle
        dataColHandle
        axes
        samplePeriod
        dt
        sampleCount
        speed
    end
    
    methods
        function obj = Controller(axes,speed)
            %CONTROLLER Constructor. Define the sampling period in ms and
            %an array of which axes to use. Enables the axes and homes.
            %   
            disp('Adding the controller Matlab library to the path.')
            run('AEROPATH.m')
            obj.handle = A3200Init();
            obj.axes = axes;
            disp('Enabling and homing X axis');
            A3200MotionEnable(obj.handle, 1, axes)
            A3200MotionHome(obj.handle, 1, axes)
            obj.speed = speed;
        end

        function home(obj)
            A3200MotionHome(obj.handle, 1, obj.axes)
        end
        
        function obj = SetDataColLength(obj,sampleCount,samplePeriod)
            %SetDataColLength Specify the number of samples to collect and
            %the time period between samples in ms.
            obj.sampleCount = sampleCount;
            obj.samplePeriod = samplePeriod;
            obj.dt = samplePeriod*.001;
            A3200Handle = obj.handle;

            % Initialize the data collection, configure the connection, and return a handle
%             disp('Creating a data collection configuration.')
            dataCollHandle = A3200DataCollectionConfigCreate(A3200Handle);
            
%             disp('Configuring to collect position feedback and velocity feedback data on the 0th axis.')
            A3200DataCollectionConfigAddSignal(dataCollHandle, A3200DataSignal.ProgramPositionFeedback, 0, 0)
            A3200DataCollectionConfigAddSignal(dataCollHandle, A3200DataSignal.VelocityFeedback, 0, 0)
            A3200DataCollectionConfigAddSignal(dataCollHandle, A3200DataSignal.ProgramPositionCommand, 0, 0)
            
%             disp('Configuring to collect one sample every one millisecond.')
            A3200DataCollectionConfigSetPeriod(dataCollHandle, obj.samplePeriod)
            
            A3200DataCollectionConfigSetSamples(dataCollHandle, sampleCount)
            obj.dataColHandle = dataCollHandle;
        end
        
        function Free(obj)
            disp('Freeing the resources used by the data collection configuration.')
            A3200DataCollectionConfigFree(obj.dataColHandle);
            disp('Disconnecting from the A3200')
            A3200Disconnect(obj.handle);
        end

        function [posData,velData,refData] = TestGain(obj, step)
            % Perform a step test at a specific gain and return collected data. Returns
            % to 0 and waits until reaches 0.
            speed = obj.speed; % use default speed
            
            % Run step
            A3200DataCollectionStart(obj.handle, obj.dataColHandle);
            A3200MotionMoveAbs(obj.handle, 1, obj.axes, step, speed); % Try Rapid command
            A3200MotionWaitForMotionDone(obj.handle, obj.axes, A3200WaitOption.InPosition, -1);
            
            % Collect Data
            collectedData = A3200DataCollectionDataRetrieve(obj.handle, obj.dataColHandle, obj.sampleCount);
            posData = Converter(collectedData(1,:));
            velData = Converter(collectedData(2,:));
            refData = Converter(collectedData(3,:));
            
            % Return to 0
            A3200MotionMoveAbs(obj.handle, 1, obj.axes, 0, speed)
            A3200MotionWaitForMotionDone(obj.handle, obj.axes, A3200WaitOption.InPosition, -1);
            end
    end
end

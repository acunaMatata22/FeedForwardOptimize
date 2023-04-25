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
        settleEpsilon
        minMove
        maxMove
    end
    
    methods
        function obj = Controller(axes,speed)
            %CONTROLLER Constructor. Define the sampling period in ms and
            %an array of which axes to use. Enables the axes and homes.
            %   
%             disp('Adding the controller Matlab library to the path.')
            run('AEROPATH.m')
            obj.handle = A3200Init();
            obj.axes = axes;
            disp('Enabling and homing X axis');
            A3200MotionEnable(obj.handle, 1, axes)
            A3200MotionHome(obj.handle, 1, axes)
            obj.speed = speed;
            obj.samplePeriod = 1;
            obj.dt = .001*obj.samplePeriod;
            obj.settleEpsilon = .10;
            obj.dataColHandle = 0;
            obj.minMove = 10^-3.5;
            obj.maxMove = 10^1.2;
        end

        function UpdateParam(obj,paramType, newGains)
            % Update parameters
            GAINAFF = A3200ParameterId.GainAff;
            GAINPFF = A3200ParameterId.GainPff;
            GAINVff = A3200ParameterId.GainVff;
            GAINJff = A3200ParameterId.GainJff;
            switch paramType
                case 'A'
                    % Only Acceleration is specified
                    assert(length(newGains) == 1, 'Wrong number of gains for A');
                    A3200ParameterSetValue(obj.handle,GAINAFF,0,newGains);
                case 'PVAJ'
                    % Position, velocity, acceleration, jerk parameters
                    assert(length(newGains) == 4, 'Wrong number of gains for PVAJ');
                    A3200ParameterSetValue(obj.handle,GAINPFF,0,newGains(1));
                    A3200ParameterSetValue(obj.handle,GAINVFF,0,newGains(2));
                    A3200ParameterSetValue(obj.handle,GAINAFF,0,newGains(3));
                    A3200ParameterSetValue(obj.handle,GAINJFF,0,newGains(4));
                otherwise
                    error('Incompatable parameter type');
            end
        end

        function Home(obj)
            A3200MotionHome(obj.handle, 1, obj.axes)
        end

        function obj = GetDataCol(obj,step)
            % Quickly get the datacollector obj. Uses default speed and
            % sample Period as per controller object.
            step_time = step / obj.speed;
            obj.sampleCount = round(1.7*step_time/(obj.dt)) + 7 + 120;
            obj = obj.SetDataColLength(obj.sampleCount,obj.samplePeriod);
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
            if obj.dataColHandle ~= 0
                disp('Freeing the resources used by the data collection configuration.')
                A3200DataCollectionConfigFree(obj.dataColHandle);
            end
            disp('Disconnecting from the A3200')
            A3200Disconnect(obj.handle);
        end

        function [posData,velData,refData] = TestGain(obj, step)
            % Perform a step test at a specific gain and return collected data. Returns
            % to 0 and waits until reaches 0.
            
            % Run step
            A3200DataCollectionStart(obj.handle, obj.dataColHandle);
            A3200MotionMoveAbs(obj.handle, 1, obj.axes, step, obj.speed); % Try Rapid command
            A3200MotionWaitForMotionDone(obj.handle, obj.axes, A3200WaitOption.InPosition, -1);
            
            % Collect Data
            collectedData = A3200DataCollectionDataRetrieve(obj.handle, obj.dataColHandle, obj.sampleCount);
            posData = Converter(collectedData(1,:));
            velData = Converter(collectedData(2,:));
            refData = Converter(collectedData(3,:));
            
            % Return to 0
            A3200MotionMoveAbs(obj.handle, 1, obj.axes, 0, obj.speed)
            A3200MotionWaitForMotionDone(obj.handle, obj.axes, A3200WaitOption.InPosition, -1);
        end

        function stepError = MeasureStep(obj, step, reference)
            [posData,~,refData] = TestGain(obj, step+reference);
            settleInd = abs(refData - (step+reference)) < obj.settleEpsilon*step;
            stepError = rmse(posData(settleInd), refData(settleInd));
        end

        function Move(obj, position)
            A3200MotionMoveAbs(obj.handle, 1, obj.axes, position, obj.speed)
            A3200MotionWaitForMotionDone(obj.handle, obj.axes, A3200WaitOption.InPosition, -1);            
        end

        function avgError = multiTest(obj, step, numTests)
            % Test multiple times with same gains and get avg error
            Errors = zeros(numTests,1);
            pd = makedist('Loguniform', 'Lower',obj.minMove, 'Upper', 1);
            randShift = random(pd,numTests,1);
            for i = 1:numTests
                % Goto -2
                obj.Move(-2);
            
                % Random move to establish backlash alternating forward/backwards
                if mod(i,2) == 0
                    start = -2 + randShift(i);
                    obj.Move(start);
                else
                    start = -2 - randShift(i);
                    obj.Move(start);
                end
            
                % Test move
                obj = obj.GetDataCol(step);
                Errors(i) = obj.MeasureStep(step, start);
            end
            avgError = sum(Errors) / numTests;

        end
    end
end


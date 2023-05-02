function updateParam(CTLR,paramType, newGains)
% Update parameters
GAINAFF = A3200ParameterId.GainAff;
GAINPFF = A3200ParameterId.GainPff;
GAINVff = A3200ParameterId.GainVff;
GAINJff = A3200ParameterId.GainJff;
switch paramType
    case 'A'
        % Only Acceleration is specified
        assert(length(newGains) == 1, 'Wrong number of gains for A');
        A3200ParameterSetValue(CTLR.handle,GAINAFF,0,newGains);
    case 'VA'
        % Position, velocity
        assert(length(newGains) == 2, 'Wrong number of gains for VA');
        A3200ParameterSetValue(CTLR.handle,GAINVFF,0,newGains(2));
        A3200ParameterSetValue(CTLR.handle,GAINAFF,0,newGains(3));
    case {'PVAJ','JAVP'}
        % Position, velocity, acceleration, jerk parameters
        assert(length(newGains) == 4, 'Wrong number of gains for PVAJ');
        A3200ParameterSetValue(CTLR.handle,GAINPFF,0,newGains(1));
        A3200ParameterSetValue(CTLR.handle,GAINVFF,0,newGains(2));
        A3200ParameterSetValue(CTLR.handle,GAINAFF,0,newGains(3));
        A3200ParameterSetValue(CTLR.handle,GAINJFF,0,newGains(4));
    otherwise
        error('Incompatable parameter type');
end
end
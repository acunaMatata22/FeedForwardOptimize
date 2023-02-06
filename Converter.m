function primaryUnits = Converter(encUnits)
    % Converts from encoder counts to the primary unit

    % Encoder counts per primary unit
    CountsPerUnit = 3276800;

    primaryUnits = encUnits / CountsPerUnit;
end
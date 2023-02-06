function handle = A3200Init()
% Add library to path and connect to controller
disp('Adding the controller Matlab library to the path.')
% Add the controller Matlab library to the path
arch = computer('arch');
if(strcmp(arch, 'win32'))
    addpath('C:\Program Files (x86)\Aerotech\A3200\Matlab\x86')
elseif(strcmp(arch, 'win64'))
    addpath('C:\Program Files (x86)\Aerotech\A3200\Matlab\x64')
end

disp('Connecting to the A3200')
handle = A3200Connect;
end
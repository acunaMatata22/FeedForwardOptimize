function f = plotResponse(f,posData,velData,refData,posColor)
% Plot Results

figure(f); hold on;
% subplot(2,1,1)
n = length(posData);
plot(1:n,posData,posColor,1:n,refData,'r');
xlabel("Time (ms)");
ylabel("Position (mm)");
% subplot(2,1,2)
% plot(velData);
% xlabel("Time (ms)");
% ylabel("Velocity (mm/s)");
% hold off;
end
function gains = fitsWrapper(fits, step)
% cell arry of fits, and a step value
n = length(fits);
gains = zeros(n,1);
for i = 1:n
    gains(i) = feval(fits{i},step);
end

end
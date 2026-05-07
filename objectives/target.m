function [res] = target(pos, target)
n = size(pos, 2);
res = 0;
for i = 1:n
    res = res + norm(pos(:,i) - target);
end
res = res / n;
end


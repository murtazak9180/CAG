function [acc] = u2acc(u, n, h)
%u2acc - convert column vector u to acc matric
% Output:
%   - acc            % n x 2 x h - The sequence of control action over the horizon
%%
acc = reshape(u, [n, 2, h]);

end


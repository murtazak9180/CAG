function [X_next] = dynamics_euler_symplectic(X, U, params)
% dynamics_euler_symplextic - Applies semi-implicit euler integration to a vectorized flock state using X_P and X_V
% X: 4n x 1 state vector [px; py; vx; vy]
% U: 2n x 1 input vector [ax; ay]
% dt: time step

% Unpack params
dt = params.dt;
vmax = params.vmax;

n = length(X) / 4;
X_P = X(1:2*n);      % [px; py] (2n x 1)
X_V = X(2*n+1:4*n);  % [vx; vy] (2n x 1)

assert(mod(length(X),4)==0, 'X must be 4n x 1');
assert(length(U) == 2*n, 'U must be 2n x 1');

V_next = X_V + U * dt;
% % clip velocities to max speed if needed
% if nargin == 4 && ~isempty(vmax)
%     speeds = sqrt(V_next(1:n).^2 + V_next(n+1:end).^2);
%     speed_factors = min(1, vmax ./ (speeds + 1e-6)); % Avoid division by zero
%     V_next = V_next .* speed_factors;
% end

P_next = X_P + V_next * dt;

X_next = [P_next; V_next];
end


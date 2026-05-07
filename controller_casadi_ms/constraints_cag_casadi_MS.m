function g = constraints_cag_casadi_MS(x0_tilde, u_horizon_tilde, x_horizon_tilde, params, scale)
%% constraints_cag_casadi_MS - Dynamics, Accel, and Vel Constraints
import casadi.*
n = params.n;
h = params.h;
ct = params.ct;
dt = params.dt;
control_steps = uint8(ct / dt);

% Reshape flat decision variables
U_h = reshape(u_horizon_tilde, 2*n, h);
X_h = reshape(x_horizon_tilde, 4*n, h); % These are the optimized states X_1...X_h

g = [];
X_prev = x0_tilde;

for idx_h = 1:h
    U_idx = U_h(:, idx_h);
    X_guess = X_h(:, idx_h); % The optimizer's proposed state for the end of this step
    
    %% Dynamics Equality Constraints (
    X_physics_next = X_prev;
    for k = 1:control_steps
        X_physics_next = dynamics_euler_symplectic_casadi(X_physics_next, U_idx, params, scale);
    end
    
    % The gap between the guess and the physics must be zero
    % Length: 4n per step
    g = [g; X_guess - X_physics_next];
    
    %%  Acceleration Magnitude Constraints
    % Length: n per step
    ax = U_idx(1:n);
    ay = U_idx(n+1:end);
    accel_mag_sq = ax.^2 + ay.^2; 
    g = [g; accel_mag_sq]; % Should be <= 1.0
    
    %% Velocity Magnitude Constraints
    vx = X_guess(2*n + 1 : 3*n);
    vy = X_guess(3*n + 1 : 4*n);
    vel_mag_sq = vx.^2 + vy.^2;
    g = [g; vel_mag_sq]; % Should be <= 1.0
    
    % Update X_prev for the next step's continuity check
    X_prev = X_guess;
end
end
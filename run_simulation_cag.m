function [traj] = run_simulation_cag(X0, params, simNum, opt)
% Unpack parameters
n = params.n;
steps = params.steps;
ct = params.ct;
dt = params.dt;

% State: X = [px; py; vx; vy]
X = zeros(4*n, steps+1);
U = zeros(2*n, steps);
U_last = zeros(2*n, 1);
U_t = zeros(2*n*params.h, 1);

% Initialize state
X(:,1) = X0;

%% --- Controller selection ---
if isfield(params, 'controller_type')
    controller_type = params.controller_type;
else
    controller_type = 'SS'; % default
end

% Init solver based on controller type
if strcmpi(controller_type, 'SS')
    solver_data = controller_cmpc_SS_cag_init(params, opt);
elseif strcmpi(controller_type, 'MS')
    solver_data = controller_cmpc_MS_cag_init(params, opt);
else
    error('Unknown controller_type. Use "SS" or "MS".');
end

controller_run = ct / dt;

for t = 1:steps

    if mod(t, 5) == 0
        fprintf('n:%d | run:%d | step:%d/%d | t=%.1fs');
    end

    %% --- Controller call ---
    if mod(t - 1, controller_run) == 0

        pos = [X(1:n, t)'; X(n+1:2*n, t)'];
        vel = [X(2*n+1:3*n, t)'; X(3*n+1:4*n, t)'];

        tic_controller = tic;

        if strcmpi(controller_type, 'SS')
            [U_t, ~] = controller_cmpc_SS_casadi_cag(pos, vel, U_t, params, solver_data);
        elseif strcmpi(controller_type, 'MS')
            [U_t, ~] = controller_cmpc_MS_casadi_cag(pos, vel, U_t, params, solver_data);
        end

        solve_time = toc(tic_controller);

        %% --- Validate solver output ---
        U_numeric = full(U_t);
        if any(isnan(U_numeric(:))) || any(isinf(U_numeric(:)))
            nan_count = nan_count + 1;
            failed_steps(end+1) = t;
            fprintf('[WARN] Step %d: NaN/Inf in U_control\n', t);
        else
            U_last = U_t;
        end
    end

    U(:,t)   = U_last;
    X(:,t+1) = dynamics_euler_symplectic(X(:,t), U(:,t), params);

end

%% --- Extract trajectory ---
X_mat = reshape(X, n, 4, steps+1);
pos   = permute(X_mat(:, 1:2, :), [2 1 3]);
vel   = permute(X_mat(:, 3:4, :), [2 1 3]);
acc   = reshape(U, 2, n, steps);

%% --- Store output ---
traj.pos = pos;
traj.vel = vel;
traj.acc = acc;

traj.params = params;

traj.x  = squeeze(pos(1,:,:)).';
traj.y  = squeeze(pos(2,:,:)).';
traj.vx = squeeze(vel(1,:,:)).';
traj.vy = squeeze(vel(2,:,:)).';
traj.ax = squeeze(acc(1,:,:)).';
traj.ay = squeeze(acc(2,:,:)).';
end
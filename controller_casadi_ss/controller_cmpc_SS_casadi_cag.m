function [U_opt, solver_metrics] = controller_cmpc_SS_casadi_cag(pos, vel, prev_sol, params, solver_data)
%% controller_cmpc_SS_cag - CasADi Runtime Solver for Single Shooting
% Inputs:
%   pos, vel    - [2 x n] current physical positions and velocities
%   prev_sol    - previous solution for warm start (2nh x 1)
%   params      - problem parameters
%   solver_data - struct from the init function
% Outputs:
%   U_opt       - [2 x n] physical accelerations for the first time step
%   solver_metrics - struct containing fval, exitflag, etc.

import casadi.*

%% Scaling the Input State
% Get scaling factors
scale = solver_data.scale;
n = solver_data.n;
h = solver_data.h;

% Convert physical inputs (2 x n) to scaled flat vector (4n x 1)
% Order: [px_1...px_n; py_1...py_n; vx_1...vx_n; vy_1...vy_n]
pos_scaled = pos / scale.L;
vel_scaled = vel / scale.V;

x0_num = [pos_scaled(1,:)'; ... % px
          pos_scaled(2,:)'; ... % py
          vel_scaled(1,:)'; ... % vx
          vel_scaled(2,:)'];    % vy

%% Warm Start Initialization
if nargin >= 3 && ~isempty(prev_sol) && numel(prev_sol) == solver_data.Nvars
    u_init = prev_sol;
else
    % Default to zero acceleration if no previous solution exists
    u_init = zeros(solver_data.Nvars, 1);
end

%% Execute Solver
sol = solver_data.solver('x0', u_init, ...
                         'lbx', solver_data.lbx, ...
                         'ubx', solver_data.ubx, ...
                         'lbg', solver_data.lbg, ...
                         ... % ubg matches our magnitude_sq <= 1.0 constraint
                         'ubg', solver_data.ubg, ... 
                         'p', x0_num);

% Extract raw results
u_opt_flat = full(sol.x);
f_opt      = full(sol.f);
stats      = solver_data.solver.stats();

%%  Format Output Metrics
solver_metrics.fval = f_opt;
solver_metrics.exitflag = stats.success;
solver_metrics.iterations = stats.iter_count;
solver_metrics.full_u = u_opt_flat; % Return for next tick's warm start
solver_metrics.return_status = stats.return_status;

%% Extract First Step and Rescale
% u_opt_flat is [ax_1...ax_h; ay_1...ay_h]

U_first_scaled_flat = u_opt_flat(1:2*n); 

% Rescale to physical units (A)
U_opt = U_first_scaled_flat * scale.A;   % <-- FIX: keep as 2n x 1
end
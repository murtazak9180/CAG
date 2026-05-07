function [U_opt, solver_metrics] = controller_cmpc_MS_casadi_cag(pos, vel, prev_Z, params, solver_data)
%% controller_cmpc_MS_cag - CasADi Runtime Solver for Multiple Shooting
import casadi.*

scale = solver_data.scale;
n = solver_data.n;
h = solver_data.h;

%%  Scale Input State
pos_scaled = pos / scale.L;
vel_scaled = vel / scale.V;
x0_num = [pos_scaled(1,:)'; pos_scaled(2,:)'; ...
          vel_scaled(1,:)'; vel_scaled(2,:)'];

%%  Warm Start Rollout
if nargin >= 3 && ~isempty(prev_Z) && numel(prev_Z) == (solver_data.num_u + solver_data.num_x)
    Z0 = prev_Z;
else
    u_init = zeros(solver_data.num_u, 1);
    x_init = zeros(solver_data.num_x, 1);
    
    x_curr = x0_num;
    control_steps = uint8(params.ct / params.dt);
    
    for i = 1:h
        for k = 1:control_steps
            x_curr = dynamics_euler_symplectic_casadi(x_curr, zeros(2*n,1), params, scale);
        end
        x_init((i-1)*4*n + 1 : i*4*n) = full(x_curr);
    end
    
    Z0 = [u_init; x_init];
end

%% Solve
sol = solver_data.solver('x0', Z0, ...
                         'lbx', solver_data.lbx, ...
                         'ubx', solver_data.ubx, ...
                         'lbg', solver_data.lbg, ...
                         'ubg', solver_data.ubg, ...
                         'p', x0_num);

z_opt = full(sol.x);
stats = solver_data.solver.stats();

%% Metrics 
solver_metrics.fval = full(sol.f);
solver_metrics.exitflag = stats.success;
solver_metrics.iterations = stats.iter_count;
solver_metrics.full_u = z_opt(1:solver_data.num_u); % only control part
solver_metrics.return_status = stats.return_status;

%% Extract First Step 
u_first_scaled = z_opt(1:2*n);

U_opt = u_first_scaled * scale.A;   
end
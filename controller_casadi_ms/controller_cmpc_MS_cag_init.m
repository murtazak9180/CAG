function solver_data = controller_cmpc_MS_cag_init(params, opts)
%% controller_cmpc_MS_cag_init - Initializes Scaled Multiple Shooting Solver
import casadi.*

n = params.n;
h = params.h;
scale = scaling_init_cag(params); 

% Dimensions
num_u = 2 * n * h; % Controls: [ax; ay] over h
num_x = 4 * n * h; % States:  [px; py; vx; vy] over h (X_2 to X_{h+1})
Nvars = num_u + num_x;

%% Symbolic variables
x0_sym = SX.sym('x0', 4*n, 1);           % Parameter: Initial State
u_sym_vec = SX.sym('u_tilde', num_u, 1); % Variables: Controls
x_sym_vec = SX.sym('x_tilde', num_x, 1); % Variables: States 

% Reshape for logic functions
x_horizon_mat = reshape(x_sym_vec, 4*n, h);

%%  Build Cost and Constraint Functions
J_expr = cost_total_cag_casadi_MS(u_sym_vec, x_horizon_mat, params, scale);
G_expr = constraints_cag_casadi_MS(x0_sym, u_sym_vec, x_horizon_mat, params, scale);

Ct_fun = Function('Ct_fun', {u_sym_vec, x_sym_vec}, {J_expr});
G_fun  = Function('G_fun',  {x0_sym, u_sym_vec, x_sym_vec}, {G_expr});

%%  Define NLP
Z = [u_sym_vec; x_sym_vec]; % Combined decision vector
J = Ct_fun(u_sym_vec, x_sym_vec);
G = G_fun(x0_sym, u_sym_vec, x_sym_vec);

nlp = struct('x', Z, 'f', J, 'g', G, 'p', x0_sym);
solver = nlpsol('solver', 'ipopt', nlp, opts);

%%  Bounds
% Decision Variables (Z)
% Controls bounded [-1, 1], States are unconstrained in box bounds
lbz = [-ones(num_u, 1); -inf(num_x, 1)];
ubz = [ ones(num_u, 1);  inf(num_x, 1)];

% Constraint Bounds (G)
% Per step: 4n gaps (must be 0), n accel_mag (<=1), n vel_mag (<=1)
lbg_step = [zeros(4*n, 1); -inf(2*n, 1)];
ubg_step = [zeros(4*n, 1);  ones(2*n, 1)];

lbg = repmat(lbg_step, h, 1);
ubg = repmat(ubg_step, h, 1);

%%  Package Data
solver_data = struct();
solver_data.solver = solver;
solver_data.lbx = lbz;
solver_data.ubx = ubz;
solver_data.lbg = lbg;
solver_data.ubg = ubg;
solver_data.num_u = num_u;
solver_data.num_x = num_x;
solver_data.n = n;
solver_data.h = h;
solver_data.scale = scale;
solver_data.params = params;

fprintf('Created scaled MS Flocking solver data for n=%d, h=%d\n', n, h);
end
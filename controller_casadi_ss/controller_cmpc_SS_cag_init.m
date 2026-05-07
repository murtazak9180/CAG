function solver_data = controller_cmpc_SS_cag_init(params, opts)
%% controller_cmpc_SS_cag_init - Initializes Scaled Single Shooting Solver
% This builds the symbolic graph and compiles the NLP once.
import casadi.*

%%  Setup Scaling
scale = scaling_init_cag(params); 

n = params.n;
h = params.h;
Nvars = 2 * n * h; % Control horizon variables [ax_1...ax_h; ay_1...ay_h]


%% Symbolic Building Blocks
%symbolic variables
x_sym_vec = SX.sym('x_tilde', 4*n, 1); %initial state
u_sym_vec = SX.sym('u_tilde', 2*n*h, 1); %controls over the horizon

% Cost Function 
% logic is derived from cost_total_cag_casadi_SS.m
J_expr = cost_total_cag_casadi_SS(x_sym_vec, u_sym_vec, params, scale);
Ct_fun = Function('Ct_fun', {x_sym_vec, u_sym_vec}, {J_expr});

% Define the Constraint Function 
G_expr = constraints_cag_casadi_SS(x_sym_vec, u_sym_vec, params, scale);
G_fun = Function('G_fun', {x_sym_vec, u_sym_vec}, {G_expr});

%% Define the NLP
% These are the symbols the solver specifically interacts with
U_solver_sym = SX.sym('U', Nvars, 1);
P_solver_sym = SX.sym('x0_p', 4*n, 1); % Initial state is a parameter 'p'

% Evaluate the pre-built logic on these symbols
J = Ct_fun(P_solver_sym, U_solver_sym);
G = G_fun(P_solver_sym, U_solver_sym);

% Construct NLP structure
nlp = struct('x', U_solver_sym, 'f', J, 'g', G, 'p', P_solver_sym);
solver = nlpsol('solver', 'ipopt', nlp, opts);

%%  Decision Variable Bounds 
% In scaled units, max acceleration magnitude check is in G, 
% but we set box bounds to [-1, 1] for individual components.
lbx = -ones(Nvars, 1);
ubx =  ones(Nvars, 1);

%%  Constraint Bounds
% G contains [accel_mag_sq; vel_mag_sq] for each agent over the horizon.
% Since magnitude_sq <= 1.0:
m_acc = n * h;
m_vel = n * h;
m_tot = m_acc + m_vel;

lbg = -inf(m_tot, 1); 
ubg =  ones(m_tot, 1); % mag^2 must be <= 1.0

%% Package solver_data
solver_data = struct();
solver_data.solver = solver;
solver_data.lbx = lbx;
solver_data.ubx = ubx;
solver_data.lbg = lbg;
solver_data.ubg = ubg;
solver_data.Nvars = Nvars;
solver_data.n = n;
solver_data.h = h;
solver_data.scale = scale;
solver_data.params = params;
solver_data.ng = m_tot;

% Initialize an empty history struct
solver_data.history.fval = [];
solver_data.history.iter = [];

fprintf('Created scaled SS Flocking solver data for n=%d, h=%d\n', n, h);

end
function x_next_tilde = dynamics_euler_symplectic_casadi(x_tilde, u_tilde, params, scale)
%% dynamics_euler_symplectic_casadi
% x_tilde: Scaled current state (4n x 1) [px; py; vx; vy]
% u_tilde: Scaled current control (2n x 1) [ax; ay]
% params:  Problem parameters
% scale:   Scaling factors from scaling_init_cag

import casadi.*
n = params.n;

%% Unscale to Physical Units
% Extract and multiply by characteristic scales
p_x = x_tilde(1:n)       * scale.L;
p_y = x_tilde(n+1:2*n)   * scale.L;
v_x = x_tilde(2*n+1:3*n) * scale.V;
v_y = x_tilde(3*n+1:4*n) * scale.V;

a_x = u_tilde(1:n)       * scale.A;
a_y = u_tilde(n+1:2*n)   * scale.A;

X_P = [p_x; p_y];
X_V = [v_x; v_y];
U   = [a_x; a_y];

%% Apply Physical Dynamics (Symplectic Euler)
% v_{k+1} = v_k + a_k * dt
% p_{k+1} = p_k + v_{k+1} * dt
V_next = X_V + U * params.dt;
P_next = X_P + V_next * params.dt;

%% Scale back down 
x_next_tilde = [P_next / scale.L; 
                V_next / scale.V];

end
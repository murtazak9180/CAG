function g = constraints_cag_casadi_SS(x0_tilde, u_horizon_tilde, params, scale)
%% Single Shooting Constraints 

import casadi.*

n = params.n;
h = params.h;
ct = params.ct;
dt = params.dt;

control_steps = uint8(ct / dt);

U_h = reshape(u_horizon_tilde, 2*n, h);

g = [];

Xk = x0_tilde;

for idx_h = 1:h

    Uk = U_h(:, idx_h);

    %% acceleration constraint
    ax = Uk(1:n);
    ay = Uk(n+1:2*n);

    g = [g; ax.^2 + ay.^2];

    %% propagate state
    X_next = Xk;

    for k = 1:control_steps
        X_next = dynamics_euler_symplectic_casadi(X_next, Uk, params, scale);
    end

    Xk = X_next;

    %% velocity constraint
    vx = Xk(2*n+1 : 3*n);
    vy = Xk(3*n+1 : 4*n);

    g = [g; vx.^2 + vy.^2];

end

end
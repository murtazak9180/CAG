function J = cost_total_cag_casadi_SS(x0_tilde, u_horizon_tilde, params, scale)
%% Single Shooting Cost 
import casadi.*
n = params.n;
h = params.h;
ct = params.ct;
dt = params.dt;
control_steps = uint8(ct / dt);
U_h = reshape(u_horizon_tilde, 2*n, h);
J = 0;
Xk = x0_tilde;

eps_sep = max(scale.dmin2 * 0.1, 1e-6);  % regularization: 10% of dmin^2, at least 1e-6
eps_tgt = 1e-6;                           % regularization for sqrt at target

for idx_h = 1:h
    Uk = U_h(:, idx_h);

    % propagate state forward
    X_next = Xk;
    for k = 1:control_steps
        X_next = dynamics_euler_symplectic_casadi(X_next, Uk, params, scale);
    end
    Xk = X_next;

    %% extract position
    px = Xk(1:n);
    py = Xk(n+1:2*n);
    pos = [px, py]';

    %% separation (1/dist^2 with epsilon regularization to prevent NaN gradient)
    res_sep = 0;
    count = n*(n-1)/2;
    for i = 1:n-1
        for j = i+1:n
            d = pos(:,i) - pos(:,j);
            dist_sq = d(1)^2 + d(2)^2;
            res_sep = res_sep + 1 / (dist_sq + eps_sep);
        end
    end
    res_sep = res_sep / count;

    %% cohesion
    res_coh = 0;
    for i = 1:n-1
        for j = i+1:n
            d = pos(:,i) - pos(:,j);
            res_coh = res_coh + (d(1)^2 + d(2)^2);
        end
    end
    res_coh = res_coh / count;

    %% target (sqrt regularized to prevent undefined gradient at exact target)
    tgt = scale.target(:);
    res_tgt = 0;
    for i = 1:n
        d = pos(:,i) - tgt;
        res_tgt = res_tgt + sqrt(d(1)^2 + d(2)^2 + eps_tgt);
    end
    res_tgt = res_tgt / n;

    J = J + params.ws*res_sep + params.wc*res_coh + params.wt*res_tgt;
end
J = J / h;
end
function J = cost_total_cag_casadi_MS(u_horizon_tilde, x_horizon_tilde, params, scale)
import casadi.*
n = params.n;
h = params.h;
J = 0;

% stability constants
eps_sep = max(scale.dmin2 * 0.1, 1e-6); 
eps_tgt = 1e-6;

for idx_h = 1:h
    X_curr = x_horizon_tilde(:, idx_h);
    
    px = X_curr(1:n);
    py = X_curr(n+1:2*n);
    pos = [px, py]'; 
    
    count = n * (n - 1) / 2;
    
    %% separation
    res_sep = 0;
    for i = 1:n-1
        for j = i+1:n
            dist_sq = sum((pos(:,i) - pos(:,j)).^2);
            res_sep = res_sep + 1 / (dist_sq + eps_sep);  %the stablization term incase the inter agent distance becomes 0. 
        end                                               %casadi reports gradients as NaNs otherwise. 
    end
    res_sep = res_sep / count;
    
    %% cohesion
    res_coh = 0;
    for i = 1:n-1
        for j = i+1:n
            res_coh = res_coh + sum((pos(:,i) - pos(:,j)).^2);
        end
    end
    res_coh = res_coh / count;
    
    %% target seeking 
    res_tgt = 0;
    tgt = scale.target(:); %column vector to match pos
    for i = 1:n
        dist_sq_tgt = sum((pos(:,i) - tgt).^2);
        res_tgt = res_tgt + sqrt(dist_sq_tgt + eps_tgt);  %yet again, sqrt stability. 
    end
    res_tgt = res_tgt / n;
    
    J = J + params.ws * res_sep + params.wc * res_coh + params.wt * res_tgt;
end

J = J / h;
end
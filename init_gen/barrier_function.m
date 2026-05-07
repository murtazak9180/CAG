function [res] = barrier_function(delta_p_ij, delta_v_ij, params)
% mode 1: normal decentralizded
res = sqrt(4 * params.amax * (norm(delta_p_ij) - params.dmin)) + ...
    dot(delta_p_ij, delta_v_ij) / norm(delta_p_ij);

res = real(res);

end


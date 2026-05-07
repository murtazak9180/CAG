function [ pos, vel ] = gen_state_random(params )
n = params.n;
minP = params.minP;
maxP = params.maxP;
minV = params.minV;
maxV = params.maxV;
%
% pos = (maxP - minP) * rand(2, n) + minP*ones(2, n);
% vel = (maxV - minV) * rand(2, n) + minV*ones(2, n);
pos = zeros(2, n);
vel = zeros(2, n);
disconnected = true;
while disconnected 
    for i = 1:n
        irrecoverable = true;
        D = 0;
        % Make sure that i) The new point is safe distance away and ii) the
        % state is recoverable.
        while D <= params.dmin || irrecoverable
            irrecoverable = false;
            if i >= 2
                pos(:, i) = (maxP - minP) * rand(2, 1) + minP*ones(2, 1);
            end
            vel(:, i) = (maxV - minV) * rand(2, 1) + minV*ones(2, 1);
            if i == 1
                break
            end
            D = inf;
            for j = 1:i-1
                delta_p_ij = pos(:,i) - pos(:,j);
                delta_v_ij = vel(:,i) - vel(:,j);
                h_ij = barrier_function(delta_p_ij, delta_v_ij, params);
                if h_ij < 0
                    irrecoverable = true;
                    break
                end
                temp = norm(pos(:,j) - pos(:,i));
                if temp <= D
                    D = temp;
                end
            end       
        end
%         disp(num2str(i));
    end
    pnet = proximityNet(pos',params.rs);
    CC = connectedComponents(pnet);
    if CC == 1
        disconnected = false;
%     else
%         disp(['disconnected: ' num2str(CC)]);
    end
end
end


function p_net = proximityNet(q,r)
N = size(q,1);
p_net = zeros(N,N);
for i=1:N
    for j=i+1:N
        if(norm(q(i,:)-q(j,:))<r)
            p_net(i,j) = 1;
            p_net(j,i) = 1;
        end
    end
end
end
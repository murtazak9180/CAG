function [cc,bins] = connectedComponents(pnet)
    G = graph(pnet);
    bins = conncomp(G);
    cc = max(bins);
end
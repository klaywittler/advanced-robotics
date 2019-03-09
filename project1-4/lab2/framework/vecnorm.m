function n = vecnorm(A, p, dim)
    n = sum(A.^p, dim).^(1/p);
end

function [out] = hasnan(A)
out = (sum(isnan(A(:)))>0);
end
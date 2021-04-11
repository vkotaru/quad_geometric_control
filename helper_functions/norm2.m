function [norm_] = norm2(A,varargin)
% norm2 calculates column/row wise norm
% arguments:    A (matrix), dim(optional)
% default dim=1
%
% output: if dim = 1, returns a row vector with norms cacluated along
% columns
% if dim =2, returns a column vector with norms calcuated along rows


if nargin > 1
    dim = varargin{1};
else
    dim = 1;
end

norm_ = sqrt(sum(A.^2,dim));

end
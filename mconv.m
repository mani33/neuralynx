function C = mconv(A,B,shape)
% mconv(param1,paramVal1,param2,paramVal2,...)
%-----------------------------------------------------------------------------------------
% MCONV - Convolves A and B exactly like matlab's conv function but when A is a matrix,
% each column of A is convolved with B.
%
% example: mconv(2dmatrix, smooth_kernal)
%
% This function is called by:
% This function calls:
% MAT-files required:
%
% See also: 

% Author: Mani Subramaniyan
% Date created: 2013-04-12
% Last revision: 2013-04-12
% Created in Matlab version: 8.1.0.604 (R2013a)
%-----------------------------------------------------------------------------------------
if nargin < 3
    shape = 'same';
end

s = size(A);
% If A is a 2d matrix:
if prod(s) > max(s)
    for j = 1:s(2)
        C(:,j) = conv(A(:,j),B,shape); %#ok
    end
else
    C = conv(A,B,shape);
end
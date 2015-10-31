function gw = getGausswin(sig_msec,bin_width_msec,varargin)
% gw = getGausswin(sig_msec,bin_width_msec,param1,paramVal1,param2,paramVal2,...)
%-----------------------------------------------------------------------------------------
% GETGAUSSWIN - Gives a normalized gaussian window for smoothing psth. 
%
% example: gw = getGausswin(25,2)
%
% This function is called by:
% This function calls: gausswin
% MAT-files required:
%
% See also: gausswin

% Author: Mani Subramaniyan
% Date created: 2012-07-05
% Last revision: 2012-07-05
% Created in Matlab version: 7.14.0.739 (R2012a)
%-----------------------------------------------------------------------------------------

% sig_msec = bin_width_msec * N / (2*alpha)
% Look at help of gausswin to see how the above was derived

nStd = 6;
N = round(nStd * sig_msec / bin_width_msec);
alpha = bin_width_msec * N * 1 / (2*sig_msec);
gw = gausswin(N,alpha);
gw = gw/sum(gw);


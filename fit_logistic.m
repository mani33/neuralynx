function [b,yhat,modelfun] = fit_logistic(x,y,b0)
% function [b,yhat,modelfun] = fit_logistic(x,y,b0)
%
% Fit a logistic function to the given data
%
% Inputs:
% y is n-by-p matrix of response variable of n observations at p different
% values of the predictor
% x is p-by-1 vector of predictor values
% Guess b0 = [ymax k x0];
%
% logistic function fitted: y = ymax/(1+exp(-k*(x-x0)) where ymax is upper
% bound of the curve, k is the slope at mid point of the curve and x0 is
% the x-coordinate of the mid point of the curve.

% We will parametrize ymax, k and x0.

modelfun = @(b,x) b(1)./(1+exp(-b(2)*(x-b(3))));
b = nlinfit(x,y,modelfun,b0);
yhat = modelfun(b,x);

end
function xm = addnoise(x, ctrlParams)
sz = size(x);
sig = ctrlParams.sigma;
xm = x + sig*randn(sz); %max(x,[],"all")*
% disp("addnoise")
% disp(xm)
% disp(xm-x)


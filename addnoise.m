function x = addnoise(x, ctrlParams)
sz = size(x);
sig = ctrlParams.sigma;
x = x + sig*randn(sz);

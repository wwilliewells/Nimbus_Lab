function [ ] = PID(P,I,D,f)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
x = 0;
xt = 20;
ip = 0;
etp = 0;

for i= 1:1/f:10
    et = xt - x;

    x = P*et + I*(ip+et) + D*(et -etp)
    etp = et; ip = ip+et;
end
end


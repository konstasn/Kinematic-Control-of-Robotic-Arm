function z = lpf(x,y,fc,Ts)
    z = (ones(size(y))-Ts*fc).*y + Ts*fc.*x;
end
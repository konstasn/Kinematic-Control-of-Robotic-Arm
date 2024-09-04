function out = satq(in,lim)
   out = [sat(in(1),lim(1));
          sat(in(2),lim(2));
          sat(in(3),lim(3));
          sat(in(4),lim(4));
          sat(in(5),lim(5));
          sat(in(6),lim(6))];
end

function satx = sat(x,a)
    if x>=a
        satx = a;
    elseif x<=-a
        satx = -a;
    else
        satx = x;
    end
end
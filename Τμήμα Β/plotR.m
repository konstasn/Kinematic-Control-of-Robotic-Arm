function [] = plotR(R_be,Ts,N)
    tiles = tiledlayout(2,3);
    title(tiles,'R_{be} as equivelant axis-angle representation')

    nexttile
    plot(Ts*(0:(N-1)),R_be(2,1:N))
    hold on
    yline(0,'--')
    hold off
    title('k_x')

    nexttile
    plot(Ts*(0:(N-1)),R_be(3,1:N))
    hold on
    yline(1,'--')
    hold off
    title('k_y')

    nexttile
    plot(Ts*(0:(N-1)),R_be(4,1:N))
    hold on
    yline(0,'--')
    hold off
    title('k_z')

    nexttile([1 3])
    plot(Ts*(0:(N-1)),R_be(1,1:N))
    hold on
    yline(180,'--')
    hold off
    title('θ')
end
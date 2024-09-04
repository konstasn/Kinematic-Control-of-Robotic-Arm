function [] = plotp(P_be,Ts,N)
    tiles1 = tiledlayout(3,1);

    nexttile
    plot(Ts*(0:(N-1)),P_be(1,1:N))
    hold on
    yline(0,'--')
    hold off
    title("p_{BE_{x}}")

    nexttile
    plot(Ts*(0:(N-1)),P_be(2,1:N))
    hold on
    yline(0,'--')
    hold off
    title("p_{BE_{y}}")

    nexttile
    plot(Ts*(0:(N-1)),P_be(3,1:N))
    hold on
    yline(0.45,'--')
    hold off
    title("p_{BE_{z}}")
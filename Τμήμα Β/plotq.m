function [] = plotq(Q_arm,Ts,N)
    tiles = tiledlayout(3,2);
    nexttile
    plot(Ts*(0:(N-1)),Q_arm(1,:))
    title("q_1")
    nexttile
    plot(Ts*(0:(N-1)),Q_arm(2,:))
    title("q_2")
    nexttile
    plot(Ts*(0:(N-1)),Q_arm(3,:))
    title("q_3")
    nexttile
    plot(Ts*(0:(N-1)),Q_arm(4,:))
    title("q_4")
    nexttile
    plot(Ts*(0:(N-1)),Q_arm(5,:))
    title("q_5")
    nexttile
    plot(Ts*(0:(N-1)),Q_arm(6,:))
    title("q_6")
end
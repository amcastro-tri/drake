d1=importdata('single_point_impact_0p125_a1em6.dat');
d2=importdata('single_point_impact_0p225_a1em6.dat');
d3=importdata('single_point_impact_0p325_a1em6.dat');
d4=importdata('single_point_impact_1p000_a1em6.dat');

d3b=importdata('single_point_impact_0p325_a1p0.dat');

d3c=importdata('single_point_impact_0p325_a1em6_hmin_1em4.dat');

% Smaller values of vs.
d5=importdata('single_point_impact_0p325_a1p0_vs1em6.dat');

h = plot(d1(:,3), d1(:,4), d2(:,3), d2(:,4), d3(:,3), d3(:,4),...
    d3b(:,3), d3b(:,4),...
    d3c(:,3), d3c(:,4),...
    d5(:,3), d5(:,4));
axis([-5 0 0 4])
set(h, 'linewidth',2)
set(gca, 'Fontname', 'Times', 'fontsize', 16)
xlabel('v_x [m/s]', 'Fontname', 'Times', 'fontsize', 16)
ylabel('v_y [m/s]', 'Fontname', 'Times', 'fontsize', 16)
legend('\mu=0.125 (a=10^{-6})', '\mu=0.225 (a=10^{-6})', ...
       '\mu=0.325 (a=10^{-6})', '\mu=0.325 (a=1)', ...
       '\mu=0.325 (a=10^{-6}, h_{min}=10^{-4})',...
       '\mu=0.325 (a=1, vs=10^{-6})');

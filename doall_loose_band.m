% All sims run with full Newton.

% =========================================================================
% Sims using a = 1e-6, no loose band.
vs = [1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 5e-7];
steps = [6973, 6947, 6919, 6912, 19555, 61464];
evals = [582040, 581911, 579455, 580157, 3463800, 12337569];
dt_min = ...
[1.67185e-09, 5.46292e-10, 1.80249e-10, 1.12918e-10, 1.46045e-10, 4.5639e-11];
dt_max = ...
[9.92552e-07, 9.07094e-07, 8.92095e-07, 8.94089e-07, 8.8571e-07, 7.22334e-07];

figure(1)
h = loglog(vs, steps, '-o');
set(h, 'linewidth', 2)
set(gca, 'Fontname', 'Times', 'fontsize', 16)
xlabel('v_s [m/s]', 'Fontname', 'Times', 'fontsize', 16)
ylabel('# Steps [-]', 'Fontname', 'Times', 'fontsize', 16)
title('a = 1e-6', 'Fontname', 'Times', 'fontsize', 16)

% =========================================================================
% Sims using a = 1e-5, no loose band.
vs = [1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 5e-7];
steps = [2226, 2186, 2172, 2177, 16718, 57718];
evals = [205979, 201949, 200826, 202634, 3250718, 11524949];
dt_min = ...
[5.491e-09, 1.82807e-09, 9.54974e-10, 2.5124e-09, 1.19801e-11,2.39601e-11];
dt_max = ...
[2.53096e-06, 2.90869e-06, 2.90601e-06, 2.88458e-06, 2.73045e-06,2.75877e-06];

figure(2)
h = loglog(vs, steps, '-o');
set(h, 'linewidth', 2)
set(gca, 'Fontname', 'Times', 'fontsize', 16)
xlabel('v_s [m/s]', 'Fontname', 'Times', 'fontsize', 16)
ylabel('# Steps [-]', 'Fontname', 'Times', 'fontsize', 16)
title('a = 1e-5', 'Fontname', 'Times', 'fontsize', 16)

% =========================================================================
% Sims using vs = 1e-4, no loose band.
acc = [1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 0.1, 0.5];
steps = [21691, 6919, 2172, 701, 227, 79, 34, 25];

figure(3)
h = loglog(acc, steps, '-o', acc, sqrt(1./acc), '--');
set(h, 'linewidth', 2)
set(gca, 'Fontname', 'Times', 'fontsize', 16)
xlabel('a [-]', 'Fontname', 'Times', 'fontsize', 16)
ylabel('# Steps [-]', 'Fontname', 'Times', 'fontsize', 16)
legend('ImplcitEuler', '2^{nd} order ref.')
title('vs = 1e-4 m/s', 'Fontname', 'Times', 'fontsize', 16)

% =========================================================================
% Loose band. Sims using vs = 1e-4, a = 1e-5, a_band = 0.5
h_band = [1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2];
steps = [2172, 2178, 2062, 499, 64, 24, 24, 24];

figure(4)
h = loglog(h_band, steps, '-o');
set(h, 'linewidth', 2)
set(gca, 'Fontname', 'Times', 'fontsize', 16)
xlabel('h_{band} [-]', 'Fontname', 'Times', 'fontsize', 16)
ylabel('# Steps [-]', 'Fontname', 'Times', 'fontsize', 16)
title('vs = 1e-4 m/s, a = 1e-5, a_{band} = 0.5', 'Fontname', 'Times', 'fontsize', 16)


% =========================================================================
% Histogram of time steps.
% vs = 1e-4, a = 1e-5.
d=importdata('steps_hist.dat');
step_size = abs(d(2:end,end)); %diff(d(:,1));
th = 0.5 * (d(1:end-1,1) + d(2:end,1));

x = th;
y = step_size;

% There seem to be two time scales???
% Maybe one for the normal stiffness and one for the regularized friction?
figure(5)
nbins = 25;
step_size_edges = logspace(log10(min(step_size)/2), log10(max(step_size)), nbins);
hh = histogram(step_size, step_size_edges);
set(gca,'xscale','log')

%set(h, 'linewidth', 2)
set(gca, 'Fontname', 'Times', 'fontsize', 16)
xlabel('h [-]', 'Fontname', 'Times', 'fontsize', 16)
ylabel('# Steps, frequency', 'Fontname', 'Times', 'fontsize', 16)
title('vs = 1e-4 m/s, a = 1e-5', 'Fontname', 'Times', 'fontsize', 16)


% =========================================================================
% Cumulative Histogram of time steps.
% vs = 1e-4, a = 1e-5.

hh_cum = cumsum(hh.Values);
hh_centers = 0.5 * (step_size_edges(1:end-1) + step_size_edges(2:end));

figure(6)
h = plot(hh_centers, hh_cum/hh_cum(end));
set(h, 'linewidth', 2)
set(gca, 'Fontname', 'Times', 'fontsize', 16)
xlabel('h [-]', 'Fontname', 'Times', 'fontsize', 16)
ylabel('# Steps, cumulative', 'Fontname', 'Times', 'fontsize', 16)
title('vs = 1e-4 m/s, a = 1e-5', 'Fontname', 'Times', 'fontsize', 16)


% =========================================================================
% Histogram of time steps.
% With h_loose=1e-6, a_loose=0.5
% vs = 1e-4, a = 1e-5.
d=importdata('steps_hist3.dat');
step_size = abs(d(2:end,end)); %diff(d(:,1));
th = 0.5 * (d(1:end-1,1) + d(2:end,1));

% There seem to be two time scales???
% Maybe one for the normal stiffness and one for the regularized friction?
figure(7)
nbins = 30;
step_size_edges = logspace(log10(min(step_size)/2), log10(max(step_size)), nbins);
hh = histogram(step_size, step_size_edges);
set(gca,'xscale','log')

%set(h, 'linewidth', 2)
set(gca, 'Fontname', 'Times', 'fontsize', 16)
xlabel('h [-]', 'Fontname', 'Times', 'fontsize', 16)
ylabel('# Steps, frequency', 'Fontname', 'Times', 'fontsize', 16)
title('vs = 1e-4 m/s, a = 1e-5, h_{band}=1e-6', 'Fontname', 'Times', 'fontsize', 16)


% =========================================================================
% Cumulative Histogram of time steps.
% With h_loose=1e-6, a_loose=0.5
% vs = 1e-4, a = 1e-5.

hh_cum = cumsum(hh.Values);
hh_centers = 0.5 * (step_size_edges(1:end-1) + step_size_edges(2:end));

figure(8)
h = plot(hh_centers, hh_cum/hh_cum(end));
set(h, 'linewidth', 2)
set(gca, 'Fontname', 'Times', 'fontsize', 16)
xlabel('h [-]', 'Fontname', 'Times', 'fontsize', 16)
ylabel('# Steps, cumulative', 'Fontname', 'Times', 'fontsize', 16)
title('vs = 1e-4 m/s, a = 1e-5, h_{band}=1e-6', 'Fontname', 'Times', 'fontsize', 16)


% =========================================================================
% Velocity profiles.
% vs = 1e-4, a = 1e-5.
% As a function of h_loose

d1=importdata('band1.dat');  % h_loose = 0, no band.
d2=importdata('band2.dat');  % h_loose = 1e-6
d3=importdata('band3.dat');  % h_loose = 1e-5
d4=importdata('band4.dat');  % h_loose = 1e-4
d5=importdata('band5.dat');  % h_loose = 0, a = 0.5 (highest).

figure(9)
h = plot(...
    d1(:,3), d1(:,4), '-',...
    d2(:,3), d2(:,4), '-^',...
    d3(:,3), d3(:,4), '-v',...
    d4(:,3), d4(:,4), '-s',...
    d5(:,3), d5(:,4), '-o');   
axis([-5 0 0 1.2])
set(h, 'linewidth',2)
set(gca, 'Fontname', 'Times', 'fontsize', 16)
xlabel('v_x [m/s]', 'Fontname', 'Times', 'fontsize', 16)
ylabel('v_y [m/s]', 'Fontname', 'Times', 'fontsize', 16)
legend('h_{band} = 0',...
       'h_{band} = 1e-6',...
       'h_{band} = 1e-5',...
       'h_{band} = 1e-4',...
       'h_{band} = \infty',...
       'Location','northwest');   

% =========================================================================
figure(7);  % brings last fig to front.


close all;
clear;

datadir = 'clutter_data_beta';
hydrodir = 'clutter_data_hydro';

% === Fixed parameters ===
E = 1e7;
beta = 1;
delta_list = [1e-5, 1e-4, 5e-4, 1e-3];
delta_str = {"1e-5", "1e-4", "5e-4", "1e-3"};
accuracy_list = [1e-1, 1e-2, 1e-3, 1e-4];
accuracy_str = {"1e-1", "1e-2", "1e-3", "1e-4"};

% --- global defaults for figures ---
set (0, "defaultaxesfontname", "Helvetica");
set (0, "defaultaxesfontsize", 30);
set (0, "defaulttextfontsize", 20);
set (0, "defaultlinelinewidth", 1.5);
set (0, "defaultaxeslinewidth", 1.5);

colors = lines(numel(delta_list) + 1);

% === Initialize figures ===
fig_iters = figure("position", [0, 0, 3840, 2160]);
fig_steps = figure("position", [0, 0, 3840, 2160]);
fig_cond_max = figure("position", [0, 0, 3840, 2160]);
fig_cond_mean = figure("position", [0, 0, 3840, 2160]);
fig_e_max = figure("position", [0, 0, 3840, 2160]);
fig_e_mean = figure("position", [0, 0, 3840, 2160]);
fig_step_ratio = figure("position", [0, 0, 3840, 2160]);



for di = 1:numel(delta_list)
    delta = delta_list(di);
    d_str = delta_str{di};

    total_iterations = zeros(size(accuracy_list));
    total_timesteps  = zeros(size(accuracy_list));
    max_condition    = zeros(size(accuracy_list));
    mean_condition   = zeros(size(accuracy_list));
    max_e            = zeros(size(accuracy_list));
    mean_e           = zeros(size(accuracy_list));
    step_ratio       = zeros(size(accuracy_list));

    for ai = 1:numel(accuracy_list)
        acc_str = accuracy_str{ai};
        fname_full = sprintf('%s_%g/E_1e7_d_%s_ac_%s.txt_full', datadir, beta, d_str, acc_str);
        fname_accepted = sprintf('%s_%g/E_1e7_d_%s_ac_%s.txt_accepted', datadir, beta, d_str, acc_str);

        if ~isfile(fname_full)
            warning("Missing file: %s", fname_full);
            continue;
        end

        if ~isfile(fname_accepted)
            warning("Missing file: %s", fname_accepted);
            continue;
        end

        
        data_full = load(fname_full);
        data_accepted = load(fname_accepted);
        iterations     = data_full(:,3);
        max_condition_ = data_accepted(:,4);
        last_condition = data_accepted(:,5);
        max_e0         = data_accepted(:,6);
        mean_e0        = data_accepted(:,7);

        total_iterations(ai) = sum(iterations);
        total_timesteps(ai)  = numel(max_e0);
        max_condition(ai)    = max(max_condition_);
        mean_condition(ai)   = mean(last_condition);
        max_e(ai)            = max(max_e0(max_e0 > 0));
        mean_e(ai)           = mean(mean_e0(mean_e0 > 0));
        step_ratio(ai)       = (length(data_full) - length(data_accepted)) / length(data_accepted);
    end

    % === Plot for this delta ===
    figure(fig_iters);
    semilogx(accuracy_list, total_iterations, '-o', 'Color', colors(di,:), ...
           'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;

    figure(fig_steps);
    semilogx(accuracy_list, total_timesteps, '-o', 'Color', colors(di,:), ...
           'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;

    figure(fig_cond_max);
    loglog(accuracy_list, max_condition, '-o', 'Color', colors(di,:), ...
           'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;

    figure(fig_cond_mean);
    loglog(accuracy_list, mean_condition, '-o', 'Color', colors(di,:), ...
           'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;

    figure(fig_e_max);
    semilogx(accuracy_list, max_e, '-o', 'Color', colors(di,:), ...
             'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;

    figure(fig_e_mean);
    semilogx(accuracy_list, mean_e, '-o', 'Color', colors(di,:), ...
             'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;
    
    figure(fig_step_ratio);
    semilogx(accuracy_list, step_ratio, '-o', 'Color', colors(di,:), ...
             'DisplayName', sprintf('\\delta = %s', d_str));
    hold on;
end

total_iterations = zeros(size(accuracy_list));
total_timesteps  = zeros(size(accuracy_list));
max_condition    = zeros(size(accuracy_list));
mean_condition   = zeros(size(accuracy_list));
max_e            = zeros(size(accuracy_list));
mean_e           = zeros(size(accuracy_list));

for ai = 1:numel(accuracy_list)
    acc_str = accuracy_str{ai};
    fname_full = sprintf('%s/E_1e7_d_%s_ac_%s.txt_full', hydrodir, "5e-2", acc_str);
    fname_accepted = sprintf('%s/E_1e7_d_%s_ac_%s.txt_accepted', hydrodir, "5e-2", acc_str);

    if ~isfile(fname_full)
        warning("Missing file: %s", fname_full);
        continue;
    end

    if ~isfile(fname_accepted)
        warning("Missing file: %s", fname_accepted);
        continue;
    end

    
    data_full = load(fname_full);
    data_accepted = load(fname_accepted);
    iterations     = data_full(:,3);
    max_condition_ = data_accepted(:,4);
    last_condition = data_accepted(:,5);
    max_e0         = data_accepted(:,6);
    mean_e0        = data_accepted(:,7);

    total_iterations(ai) = sum(iterations);
    total_timesteps(ai)  = numel(max_e0);
    max_condition(ai)    = max(max_condition_);
    mean_condition(ai)   = mean(last_condition);
    max_e(ai)            = max(max_e0(max_e0 > 0));
    mean_e(ai)           = mean(mean_e0(mean_e0 > 0));
end

% === Plot for this delta ===
figure(fig_iters);
semilogx(accuracy_list, total_iterations, '-o', 'Color', colors(numel(delta_list)+1,:), ...
       'DisplayName', 'hydro');
hold on;

figure(fig_steps);
semilogx(accuracy_list, total_timesteps, '-o', 'Color', colors(numel(delta_list)+1,:), ...
       'DisplayName', 'hydro');
hold on;

figure(fig_cond_max);
loglog(accuracy_list, max_condition, '-o', 'Color', colors(numel(delta_list)+1,:), ...
       'DisplayName', 'hydro');
hold on;

figure(fig_cond_mean);
loglog(accuracy_list, mean_condition, '-o', 'Color', colors(numel(delta_list)+1,:), ...
       'DisplayName', 'hydro');
hold on;

figure(fig_e_max);
semilogx(accuracy_list, max_e, '-o', 'Color', colors(numel(delta_list)+1,:), ...
       'DisplayName', 'hydro');
hold on;

figure(fig_e_mean);
semilogx(accuracy_list, mean_e, '-o', 'Color', colors(numel(delta_list)+1,:), ...
       'DisplayName', 'hydro');
hold on;

% === Finalize figures ===
figure(fig_iters);
xlabel('Accuracy'); ylabel('Total iterations');
title('Total iterations vs accuracy');
legend('show'); legend('location', 'best', 'box', 'off');
grid on; box on;
print("accuracy_total_iterations.png", "-S1920,1080", "-r600", "-dpng");

figure(fig_steps);
xlabel('Accuracy'); ylabel('Number of steps');
title('Number of steps vs accuracy');
legend('show'); legend('location', 'best', 'box', 'off');
grid on; box on;
print("accuracy_num_steps.png", "-S1920,1080", "-r600", "-dpng");

figure(fig_cond_max);
xlabel('Accuracy'); ylabel('Max condition number');
title('Max condition number vs accuracy');
legend('show'); legend('location', 'best', 'box', 'off');
grid on; box on;
print("accuracy_condition_max.png", "-S1920,1080", "-r600", "-dpng");

figure(fig_cond_mean);
xlabel('Accuracy'); ylabel('Mean condition number');
title('Mean condition number vs accuracy');
legend('show'); legend('location', 'best', 'box', 'off');
grid on; box on;
print("accuracy_condition_mean.png", "-S1920,1080", "-r600", "-dpng");

figure(fig_e_max);
xlabel('Accuracy'); ylabel('Max e');
title('Max extent vs accuracy');
legend('show'); legend('location', 'best', 'box', 'off');
grid on; box on;
print("accuracy_e_max.png", "-S1920,1080", "-r600", "-dpng");

figure(fig_e_mean);
xlabel('Accuracy'); ylabel('Mean e');
title('Mean extent vs accuracy');
legend('show'); legend('location', 'best', 'box', 'off');
grid on; box on;
print("accuracy_e_mean.png", "-S1920,1080", "-r600", "-dpng");

figure(fig_step_ratio);
xlabel('Accuracy'); ylabel('Ratio of failed steps');
title('Failed step ratio vs accuracy');
legend('show'); legend('location', 'best', 'box', 'off');
grid on; box on;
print("accuracy_step_ratio.png", "-S1920,1080", "-r600", "-dpng");

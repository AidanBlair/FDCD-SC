% Plot combined figures
% Figure 1: average cardinality errors
function [] = plot_6cams_figures_final(truth,meas)
    D1 = load("results/scenario1_fixed_Fixed_results_30runs.mat");
    num_estimates_fixed = D1.num_estimates;
    D2 = load("results/scenario1_fixed_ISC_results_30runs.mat");
    num_estimates_single = D2.num_estimates;
    D3 = load("results/scenario1_fixed_DCD_1_results_30runs.mat");
    num_estimates_dcd_1 = D3.num_estimates;
    D4 = load("results/scenario1_fixed_DCD_5_results_30runs.mat");
    num_estimates_dcd_5 = D4.num_estimates;
    D5 = load("results/scenario1_fixed_DCD_10_results_30runs.mat");
    num_estimates_dcd_10 = D5.num_estimates;
    D6 = load("results/scenario1_fixed_DMSC_results_30runs.mat");
    num_estimates_dmsc = D6.num_estimates;
    %D7 = load("results_240824_dcd_1_30_times_6s_11t_7a.mat");
    %num_estimates_dcd_1 = D7.num_estimates;
    %[dist_fixed, loc_fixed, card_fixed, dist2_fixed, loc2_fixed, card2_fixed, times_fixed, num_estimates_fixed] = load("results_260624_fixed_30_times_6s_11t_7a.mat");
    %[dist_single, loc_single, card_single, dist2_single, loc2_single, card2_single, times_single, num_estimates_single] = load("results_060624_single_30_times_6s_11t_7a.mat");
    %[dist_flooding, loc_flooding, card_flooding, dist2_flooding, loc2_flooding, card2_flooding, times_flooding, num_estimates_flooding] = load("results_040624_flooding_30_times_6s_11t_7a.mat");
    
    ests_fixed = zeros(meas{1}.K, 1);
    ests_single = zeros(meas{1}.K, 1);
    %ests_flooding = zeros(meas{1}.K, 1);
    ests_dcd_1 = zeros(meas{1}.K, 1);
    ests_dcd_5 = zeros(meas{1}.K, 1);
    ests_dcd_10 = zeros(meas{1}.K, 1);
    %ests_dcd_29 = zeros(meas{1}.K, 1);
    ests_dmsc = zeros(meas{1}.K, 1);
    sz = meas{1}.K * size(meas, 2);
    NUM_RUNS = size(num_estimates_fixed, 1) / sz;
    
    for k = 1:meas{1}.K
        for run = 1:NUM_RUNS
            for s = 1:size(meas, 2)
                ests_fixed(k) = ests_fixed(k) + num_estimates_fixed(sz * (run-1) + meas{1}.K * (s-1) + k);
                ests_single(k) = ests_single(k) + num_estimates_single(sz * (run-1) + meas{1}.K * (s-1) + k);
                ests_dmsc(k) = ests_dmsc(k) + num_estimates_dmsc(sz * (run-1) + meas{1}.K * (s-1) + k);
            end
        end
        ests_fixed(k) = ests_fixed(k) / (NUM_RUNS * size(meas, 2));
        ests_single(k) = ests_single(k) / (NUM_RUNS * size(meas, 2));
        ests_dmsc(k) = ests_dmsc(k) / (NUM_RUNS * size(meas, 2));
    end

    NUM_RUNS_DCD = size(num_estimates_dcd_1, 1) / sz;
    for k = 1:meas{1}.K
        for run = 1:NUM_RUNS_DCD
            for s = 1:size(meas, 2)
                ests_dcd_1(k) = ests_dcd_1(k) + num_estimates_dcd_1(sz * (run-1) + meas{1}.K * (s-1) + k);
                ests_dcd_5(k) = ests_dcd_5(k) + num_estimates_dcd_5(sz * (run-1) + meas{1}.K * (s-1) + k);
                ests_dcd_10(k) = ests_dcd_10(k) + num_estimates_dcd_10(sz * (run-1) + meas{1}.K * (s-1) + k);
                %ests_dcd_29(k) = ests_dcd_29(k) + num_estimates_dcd_29(sz * (run-1) + meas{1}.K * (s-1) + k);
            end
        end
        ests_dcd_1(k) = ests_dcd_1(k) / (NUM_RUNS_DCD * size(meas, 2));
        ests_dcd_5(k) = ests_dcd_5(k) / (NUM_RUNS_DCD * size(meas, 2));
        ests_dcd_10(k) = ests_dcd_10(k) / (NUM_RUNS_DCD * size(meas, 2));
        %ests_dcd_29(k) = ests_dcd_29(k) / (NUM_RUNS_DCD * size(meas, 2));
    end
    
    %{
    %plot fixed cardinality
    figure('visible','on'); cardinality= gcf; 
    subplot(3,1,1); box on; hold on;
    stairs(1:meas{1}.K,truth.N,'k'); 
    plot(1:meas{1}.K,ests_fixed,'k.');
    
    grid on;
    legend(gca,'True','Estimated', 'Location', 'southeast');
    set(gca, 'XLim',[1 meas{1}.K]); set(gca, 'YLim',[0 max(truth.N)+1]);
    xlabel('Time'); ylabel('Cardinality');
    title("Fixed Sensors")

    % plot individual cardinality
    subplot(3,1,2); box on; hold on;
    stairs(1:meas{1}.K,truth.N,'k'); 
    plot(1:meas{1}.K,ests_single,'k.');
    
    grid on;
    legend(gca,'True','Estimated', 'Location', 'southeast');
    set(gca, 'XLim',[1 meas{1}.K]); set(gca, 'YLim',[0 max(truth.N)+1]);
    xlabel('Time'); ylabel('Cardinality');
    title("Individual Sensor Control (I-SC)")

    % plot flooding cardinality
    subplot(3,1,3); box on; hold on;
    stairs(1:meas{1}.K,truth.N,'k'); 
    plot(1:meas{1}.K,ests_flooding,'k.');
    
    grid on;
    legend(gca,'True','Estimated', 'Location', 'southeast');
    set(gca, 'XLim',[1 meas{1}.K]); set(gca, 'YLim',[0 max(truth.N)+1]);
    xlabel('Time'); ylabel('Cardinality');
    title("Distributed Flooding Sensor Control (DF-SC)")
    %}

    %plot all on single figure
    figure('visible','on'); cardinality= gcf; 
    box on; hold on;
    s = stairs(1:meas{1}.K,truth.N);
    s.LineWidth = 5.0;
    s.Color = 'k';
    plot(1:meas{1}.K,ests_fixed,'k. -', 'LineWidth', 2.0);
    plot(1:meas{1}.K,ests_single,'r.-', 'LineWidth', 2.0);
    plot(1:meas{1}.K,ests_dcd_1,'c.-', 'LineWidth', 2.0);
    plot(1:meas{1}.K,ests_dcd_5,'m.-', 'LineWidth', 2.0);
    plot(1:meas{1}.K,ests_dcd_10,'b.-', 'LineWidth', 2.0);
    plot(1:meas{1}.K,ests_dmsc,'g.-', 'LineWidth', 2.0);
    %plot(1:meas{1}.K,ests_dcd_29,'b-');
    
    grid on;
    % 'DCD-SC (1)', 'DCD-SC (5)', 'DCD-SC (10)',
    legend(gca,'Truth','Fixed', 'I-SC', 'DCD-SC (1)', 'DCD-SC (5)', 'DCD-SC (10)', 'FDCD-SC', 'Location', 'southeast');
    set(gca, 'XLim',[1 meas{1}.K]); set(gca, 'YLim',[0 max(truth.N)+1]);
    xlabel('Time Steps'); ylabel('Cardinality');
    %title("Cardinality")
    
    saveas(gcf, "plot_6cams_cardinality_comparison_final.pdf")
end
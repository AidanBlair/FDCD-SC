function [] = plot_average_cardinality_error(truth,meas,num_estimates)

ests = zeros(meas{1}.K, 1);
sz = meas{1}.K * size(meas, 2);
NUM_RUNS = size(num_estimates, 1) / sz;

for k = 1:meas{1}.K
    for run = 1:NUM_RUNS
        for s = 1:size(meas, 2)
            ests(k) = ests(k) + num_estimates(sz * (run-1) + meas{1}.K * (s-1) + k);
        end
    end
    ests(k) = ests(k) / (NUM_RUNS * size(meas, 2));
end

%plot cardinality
figure('visible','on'); cardinality= gcf; 
subplot(2,1,1); box on; hold on;
stairs(1:meas{1}.K,truth.N,'k'); 
plot(1:meas{1}.K,ests,'k.');

grid on;
legend(gca,'True','Estimated', 'Location', 'southeast');
set(gca, 'XLim',[1 meas{1}.K]); set(gca, 'YLim',[0 max(truth.N)+1]);
xlabel('Time'); ylabel('Cardinality');

end
close all

T = readtable('C:\Users\Andrey\Documents\UKFRepo\UnscentedKalmanFilter\sims.csv');

time = T.Time_s_;
z = T.PositionZ_m_;

hold on
plot(time,z, 'g-');
hold off

[p,~,mu] = polyfit(T.Time_s_, T.PositionZ_m_, 200);

f = polyval(p,T.Time_s_,[],mu);

ylim([0 3000])

hold on
plot(T.Time_s_,f)

% Plot the Residuals to Evaluate the Fit
plot(f,time,z,'residuals');
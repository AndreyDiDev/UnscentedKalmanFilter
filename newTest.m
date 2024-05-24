clc;
clear;
% y = [4.0432,4.1073,4.0899,4.1319,4.2885,4.4305,4.5249,4.6172,4.6962,4.7235,4.5987,4.7927,4.895,4.9079];
% x = [1999,2000,2001,2002,2003,2004,2005,2006,2007,2008,2009,2010,2011,2012];
T = readtable('C:\Users\Andrey\Documents\UKFRepo\UnscentedKalmanFilter\sims.csv');

time = T.Time_s_;
z = T.PositionZ_m_;

y = z;
x = time;

% Fit a polynomial p of degree "degree" to the (x,y) data:
degree = 100;
p = polyfit(x,y,degree);

% Evaluate the fitted polynomial p and plot:
f = polyval(p,x);

% Calculate residuals
actual_values= z;
predicted_values = f;

residuals = actual_values - predicted_values;

eqn = poly_equation(flip(p)); % polynomial equation (string)
Rsquared = my_Rsquared_coeff(y,f); % correlation coefficient

figure(3);plot(x,y,'o',x,f,'-')
legend('data',eqn)
title(['Data fit - R squared = ' num2str(Rsquared)]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% for residual graph
% Create a scatter plot
figure;
scatter(1:length(residuals), residuals, 'filled');
xlabel('Observation Index');
ylabel('Residuals');
title('Residual Plot');
grid on;

% Optionally, add a horizontal line at y = 0
hold on;
plot([1, length(residuals)], [0, 0], 'r--');
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Rsquared = my_Rsquared_coeff(data,data_fit)
    % R2 correlation coefficient computation
    
    % The total sum of squares
    sum_of_squares = sum((data-mean(data)).^2);
    
    % The sum of squares of residuals, also called the residual sum of squares:
    sum_of_squares_of_residuals = sum((data-data_fit).^2);
    
    % definition of the coefficient of correlation is
    Rsquared = 1 - sum_of_squares_of_residuals/sum_of_squares;
    
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function eqn = poly_equation(a_hat)
eqn = " y = "+a_hat(1);
for i = 2:(length(a_hat))
    if sign(a_hat(i))>0
        str = " + ";
    else
        str = " ";
    end
    if i == 2
        eqn = eqn+str+a_hat(i)+"*x";
    else
        eqn = eqn+str+a_hat(i)+"*x^"+(i-1)+" ";
    end
end
eqn = eqn+" ";
end
        
clc;
clear;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% extract
T = readtable('C:\Users\Andrey\Documents\UKFRepo\UnscentedKalmanFilter\sims.csv');

time = T.Time_s_;
z = T.PositionZ_m_;

y = z;
x = time;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% split at apogee

%n = ceil(numel(z) / 2); % number of data entries
[max,n] = max(z);

% Split into two halves
halves = mat2cell(y(:)', 1, [n, numel(y) - n]);
halvesX = mat2cell(x(:)', 1, [n, numel(x) - n]);

before = halves{1};
beforeX = halvesX{1};

after = halves{2};
afterX = halvesX{2};
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% fit halves

% Fit a polynomial p of degree "degree" to the (x,y) data:
degree = 3;

beforePolyFit = polyfit(beforeX,before,degree);
afterPolyFit = polyfit(afterX,after,degree);

%p = polyfit(x,y,degree);

% Evaluate the fitted polynomial p and plot:
%f = polyval(p,x);

% Evaluate the fitted polynomial p and plot:
beforeFunc = polyval(beforePolyFit,beforeX);
afterFunc = polyval(afterPolyFit, afterX);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% calculate performance metrics
% Calculate residuals
actual_values = z;
before_predicted_values = beforeFunc;
after_predicted_values = afterFunc;

before_residuals = actual_values - before_predicted_values;
after_residuals = actual_values - after_predicted_values;

before_Rsquared = my_Rsquared_coeff(z,beforeFunc); % correlation coefficient
after_Rsquared = my_Rsquared_coeff(z,afterFunc); % correlation coefficient

before_eqn = poly_equation(flip(beforePolyFit)); % polynomial equation (string)
after_eqn = poly_equation(flip(afterPolyFit)); % polynomial equation (string)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Figure 1 - split
%figure(1);plot(x,y,'o',x,f,'-')
figure(1);

plot(beforeX, before, 'b');
hold on;
plot(afterX, after, 'r');
hold on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% figure 2 - both fits
figure(2);

plot(beforeX, beforeFunc, 'c');
hold on;
plot(afterX, afterFunc, 'o');
hold on;
plot(time,z);
hold off;

legend('data', before_eqn);
%title(['Data fit - R squared = ' num2str(before_Rsquared) + ' After ' + num2str(after_Rsquared)]);
title(['Data fit - R squared = ' num2str(before_Rsquared) ' After ' num2str(after_Rsquared)]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% fig 3 - for residual graph
% Create a scatter plot
figure(3);
scatter(1:length(before_residuals), before_residuals, 'filled');
xlabel('Observation Index');
ylabel('Residuals');
title('Residual Plot');
grid on;

% horizontal line at y = 0
hold on;
plot([1, length(before_residuals)], [0, 0], 'r--');
hold off;

%%%%%%%%%%%%%%% fig 4 - after residual 
figure(4);
scatter(1:length(after_residuals), after_residuals, 'filled');
xlabel('Observation Index');
ylabel('Residuals');
title('After Apogee Residual Plot');
grid on;

% horizontal line at y = 0
hold on;
plot([1, length(after_residuals)], [0, 0], 'r--');
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Functions


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% getting R-sqr

function Rsquared = my_Rsquared_coeff(data,data_fit)
    % R2 correlation coefficient computation
    
    % The total sum of squares
    sum_of_squares = sum((data-mean(data)).^2);
    
    % The sum of squares of residuals, also called the residual sum of squares:
    sum_of_squares_of_residuals = sum((data-data_fit).^2);
    
    % definition of the coefficient of correlation is
    Rsquared = 1 - sum_of_squares_of_residuals/sum_of_squares;
    
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% equation writing
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
        
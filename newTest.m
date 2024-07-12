clc;
clear;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% extract
T = readtable('C:\Users\Andrey\Documents\UKFRepo\UnscentedKalmanFilter\sims.csv');

time = T.Time_s_;
z = T.PositionZ_m_;

y = z;
x = time;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% split at apogee
% have to split at max not half

%n = ceil(numel(z) / 2); % number of data entries
[n,max] = max(z);

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

p = polyfit(x,y,degree);

% Evaluate the fitted polynomial p and plot:
f = polyval(p,x);

% Calculate residuals
actual_values= z;
predicted_values = f;

residuals = actual_values - predicted_values;

eqn = poly_equation(flip(p)); % polynomial equation (string)
Rsquared = my_Rsquared_coeff(y,f); % correlation coefficient

%figure(1);plot(x,y,'o',x,f,'-')
figure(1);

plot(beforeX, before, 'b');
hold on;
plot(afterX, after, 'r');
hold on;

plot(beforeX, beforeFit, 'c');
hold on;
plot(afterX, afterFit, 'o');
hold off;

legend('data', eqn)
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
        
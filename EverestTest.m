clc;
clear;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% extract
alt_Data = readtable('C:\Users\Andrey\Downloads\Altimeter.csv');
filtered_Data = readtable('C:\Users\Andrey\Documents\EverestRepo\Apogee-Detection-Everest\EverestLibrary\everest3.txt');
baro_Data = readtable('C:\Users\Andrey\Downloads\baro_3.csv');

% altimeter
time = alt_Data.time;
true = alt_Data.altitude;

% filtered
filtered = filtered_Data.Var16;
filtered_time = filtered_Data.Var1;

% baro
baro = baro_Data.baro_pressure;

%Rsquared = my_Rsquared_coeff(true,filtered); % correlation coefficient
subplot(1,1,1)
title('Filtered vs True');
plot(time,true,'b-',filtered_time,filtered,'r-')

%figure(1);
%plot(time, true, 'c');
%hold off;
%plot(time, filtered, 'o');
%hold on;

%Rsquared;
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
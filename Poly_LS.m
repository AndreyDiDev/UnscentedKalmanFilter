close all

T = readtable('C:\Users\Andrey\Documents\UKFRepo\UnscentedKalmanFilter\sims.csv');

[p,~,mu] = polyfit(T.Time_s_, T.PositionZ_m_, 190);

f = polyval(p,T.Time_s_,[],mu);
hold on
plot(T.Time_s_,f)
hold off

% x = [T.Time_s_]';
% y = [T.PositionZ_m_]';
% 
% n = length(x);
% 
% % for example, do quadratic curve fit, can easily extend to higher degree poly
% % a = [a0 a1 a2]
% X = [ones(n,1) x x.^2];
% 
% a = inv(X'*X)*X'*y
% 
%  scatter(x,y)
% 
% % with weights
% w = [1 20 20 1];
% W = diag(w);
% 
% aweighed = inv(X'*W*X)*X'*W*y
% 
% syms xvar
% eqflat = a(1,1) + a(2,1)*xvar + a(3,1)*xvar^2;
% eqweighed = aweighed(1,1) + aweighed(2,1)*xvar + aweighed(3,1)*xvar^2;
% 
% figure
% hold on
% scatter(x,y)
% ezplot(eqflat, [0 10])
% ezplot(eqweighed, [0 10])
% legend('points', 'flat', 'weighed')
% 
% % variance y (with regular, similar for weighted)
% ypred = a(1,1) + a(2,1).*x + a(3,1)*x.^2;
% vary = 1/(n-3)*sum((y - ypred).^2)
% 
% % variance a
% Ca = vary*inv(X'*X);
% vara = diag(Ca)
% 
% % variance ypred
% varypred = [];
% Xt = X';
% for i = 1:n
%     varypred = [varypred; X(i,:)*Ca*Xt(:,i)];
% end
% varypred

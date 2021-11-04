close all
clear
clc

lookup = csvread("ntc.csv");
t = lookup(:, 1);
ad = [];

for i =1: length(lookup(:, 2))
     ad_h = (lookup(i, 2) ./ (10 + lookup(i, 2))) .* 1024;
     ad = [ad ad_h];
end
 
ad = ad';

p = polyfit(ad, t, 10);
ad2 = 0 : 1023;
t2 = round(polyval(p, ad2), 1);

hold on
plot(ad2, t2, 'r');
plot(ad, t, 'b*');
hold off

dlmwrite('data.dlm', t2*10, ',');
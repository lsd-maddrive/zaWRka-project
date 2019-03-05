% 
% SP = []; 
% wind = 10; 
% for i = 1:1:length(B)-wind
%   sp = B(i + wind, 1) - B(i, 1);
%   SP = [SP; sp];
% end
% 
% SP = SP./wind;

RARE = [];
OFTEN = [];

for i = 1:2:(length(B)-1)
   OFTEN = [OFTEN; B(i, 1)];    
end

for j = 2:2:length(B)
   RARE = [RARE; B(j, 1)];
end

plot(OFTEN)
grid on
hold on
plot(RARE)
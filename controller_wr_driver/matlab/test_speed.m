% 
% SP = []; 
% wind = 10
% for i = 1:1:length(B)-wind
%   sp = B(i + wind, 1) - B(i, 1);
%   SP = [SP; sp];
% end
% 
% SP = SP./wind;

REF = [];
SPEED = [];
T = []; 

for t = 0:1:(length(B)/2)-1
    T = [T; t*10];  
end

for i = 1:2:(length(B)-1)
   SPEED = [SPEED; B(i, 1)]; 
   
   REF = [REF; B(i+1, 1)];
   
end
% 
% for j = 2:2:length(B)
%    REF = [REF; B(j, 1)];
% end

plot(T, SPEED)
grid on
hold on
plot(T, REF)
xlabel('t, ms');
ylabel('speed, cm/s');
REF = [];
SPEED = [];
T = []; 

for i = 1:3:(length(B)-1)
   T = [T; B(i, 1)]; 
   SPEED = [SPEED; B(i, 1)]; 
   REF = [REF; B(i+1, 1)];
   
end

plot(T, SPEED)
grid on
hold on
plot(T, REF)
xlabel('t, ms');
ylabel('speed, cm/s');
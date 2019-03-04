
SP = []; 
wind = 10; 
for i = 1:1:length(B)-wind
  sp = B(i + wind, 1) - B(i, 1);
  SP = [SP; sp];
end

SP = SP./wind;
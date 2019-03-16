RAW = [];
FILT = [];
TIME = []; 

% for t = 0:1:(length(B)/2)-1
%     T = [T; t*10];  
% end

for i = 1:3:(length(B)-2)
   TIME = [TIME; B(i, 1)]; 
    
   FILT = [FILT; B(i+1, 1)]; 
   
   RAW = [RAW; B(i+2, 1)];
   
end

% TIME = TIME./1000; 
plot(TIME, FILT)
grid on
hold on
plot(TIME, RAW)
xlabel('t, ms');
ylabel('ADC');
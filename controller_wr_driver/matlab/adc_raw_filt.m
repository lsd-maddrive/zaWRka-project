RAW = [];
FILT = [];
TIME = []; 


for i = 1:3:(length(B)-2)
   TIME = [TIME; B(i, 1)]; 
   FILT = [FILT; B(i+1, 1)]; 
   RAW = [RAW; B(i+2, 1)];
end

plot(TIME, FILT)
grid on
hold on
plot(TIME, RAW)
xlabel('t, ms');
ylabel('ADC');
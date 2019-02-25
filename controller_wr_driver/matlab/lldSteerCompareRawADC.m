clear all
global dat
delete(instrfind);
dat = serial('COM9', 'BaudRate', 115200);
dat.InputBufferSize = 4096;

fopen(dat)
set(dat, 'ByteOrder', 'littleEndian')

disp 'Ok!'

start = 's';
fwrite(dat, start, 'uint8'); 

A = [];
B = [];
t = 20;

for i = 1:t
   A=fread(dat, [100,1], 'uint16');
   B = [B; A];
end
 
fclose(dat);
disp 'Finish!'


RAW_ADC = [];
MEAN_ADC = [];
for j = 1:2:length(B)-1
    RAW_ADC = [RAW_ADC; B(j, 1)];
end

for j = 2:2:length(B)
    MEAN_ADC = [MEAN_ADC; B(j, 1)];
end

hold on
plot(RAW_ADC)
plot(MEAN_ADC)
grid on 

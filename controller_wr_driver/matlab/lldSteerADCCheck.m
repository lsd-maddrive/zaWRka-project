clear all
global dat
delete(instrfind);
dat = serial('COM9', 'BaudRate', 115200);
dat.InputBufferSize = 4096;

fopen(dat)
set(dat, 'ByteOrder', 'littleEndian')

disp 'Ok!'

start = 'p';
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


plot(B)
grid on 

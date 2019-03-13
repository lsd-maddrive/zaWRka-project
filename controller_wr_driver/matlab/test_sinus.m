global dat
delete(instrfind);
dat = serial('COM9', 'BaudRate', 115200);
dat.InputBufferSize = 4096;

fopen(dat)
set(dat, 'ByteOrder', 'littleEndian')

disp 'Ok!'

start = 'p';     % start 
fwrite(dat, start, 'uint8'); 
disp 'start!'
A = [];
B = [];

t1 = 40;

for i = 1:t1
   A=fread(dat, [100,1], 'int16');
   B = [B; A];
end

start = 'f';     % start 
fwrite(dat, start, 'uint8'); 

fclose(dat);
disp 'Finish!'
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
t = 2;

for i = 1:t
   A=fread(dat, [100,1], 'int16');
   B = [B; A];
end
 
start = 'a'; %left
fwrite(dat, start, 'uint8'); 

for i = 1:t
   A=fread(dat, [100,1], 'int16');
   B = [B; A];
end

start = 'd'; %right
fwrite(dat, start, 'uint8'); 
fwrite(dat, start, 'uint8'); 

for i = 1:t
   A=fread(dat, [100,1], 'int16');
   B = [B; A];
end

start = 'a'; %right
fwrite(dat, start, 'uint8'); 
for i = 1:t
   A=fread(dat, [100,1], 'int16');
   B = [B; A];
end

fclose(dat);
disp 'Finish!'

FB = [];
REF = []; 
for i = 1: 2: (length(B) - 1)
    FB = [FB; B(i, 1)];
end
FB = FB./100;
for r = 2: 2:length(B)
   REF = [REF; B(r, 1)]; 
end

plot(FB)
grid on 
hold on
plot(REF)

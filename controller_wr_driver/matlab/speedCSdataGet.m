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

t1 = 2;

for i = 1:t1
   A=fread(dat, [100,1], 'int16');
   B = [B; A];
end
 
start = 'a'; %forward
fwrite(dat, start, 'uint8'); 

t2 = 4;
for i = 1:t2
   A=fread(dat, [100,1], 'int16');
   B = [B; A];
end

start = 's'; %backward
fwrite(dat, start, 'uint8'); 
% fwrite(dat, start, 'uint8'); 

t3 = 6;
for i = 1:t3
   A=fread(dat, [100,1], 'int16');
   B = [B; A];
end

fclose(dat);
disp 'Finish!'
% 
% % CNTRL = [];
% % SPEED = []; 
% % for i = 1: 2: (length(B) - 1)
% %     CNTRL = [CNTRL; B(i, 1)];
% % end
% % CNTRL = CNTRL./100;
% % for r = 2: 2:length(B)
% %    SPEED = [SPEED; B(r, 1)]; 
% % end
% % 
% % plot(CNTRL)
% % grid on 
% % hold on
% % plot(SPEED)
% 
% plot(B)
% grid on


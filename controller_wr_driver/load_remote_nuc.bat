set NUC=192.168.31.99

set NUC=usersaunuc-NUC7i5BNK.local

scp -P9992 build/ch.bin user-sau-nuc@%NUC%:/tmp
ssh -p9992 user-sau-nuc@%NUC% 'st-flash write /tmp/ch.bin 0x8000000'
pause

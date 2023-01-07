make clean
make ENCODER_SUPPORT=1
make clean 2S=0
make 2S=0 ENCODER_SUPPORT=1
mv head.pgh headENC.pgh

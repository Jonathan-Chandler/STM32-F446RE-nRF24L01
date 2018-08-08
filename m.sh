make clean
make
st-flash write ./build/stm32-nRF24L01.bin 0x8000000

wget https://github.com/rrivirr/rriv-firmware/releases/latest/download/rriv-firmware.bin -O ./rriv-firmware.bin
probe-rs download  ./rriv-firmware.bin \
	--chip STM32F103RE  \
 	--protocol swd \
	--allow-erase-all \
	--chip-erase
rm rriv-firmware.bin

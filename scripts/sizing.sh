ls -lah target/thumbv7m-none-eabi/debug/app
size -A -d  target/thumbv7m-none-eabi/debug/app
nm --print-size --size-sort --radix=d target/thumbv7m-none-eabi/debug/app
pip install puncover
puncover --gcc-tools-base /usr/bin  --elf-file target/thumbv7m-none-eabi/debug/ap

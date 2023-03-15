#include "rust.h"
#include "rust_serial.h"

struct SerialInterfaceContext *r_rust_serial_interface_new(void){
  return rust_serial_interface_new();
}

char r_rust_serial_read(struct SerialInterfaceContext *serial_ptr){
  return rust_serial_read(serial_ptr);
}

void r_rust_serial_write(struct SerialInterfaceContext *serial_ptr, char value){
  rust_serial_write(serial_ptr, value);
}
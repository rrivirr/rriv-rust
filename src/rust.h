struct SerialInterfaceContext *r_rust_serial_interface_new(void);

char r_rust_serial_read(struct SerialInterfaceContext *serial_ptr);

void r_rust_serial_write(struct SerialInterfaceContext *serial_ptr, char value);
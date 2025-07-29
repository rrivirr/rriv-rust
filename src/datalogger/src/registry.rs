const SENSOR_NAMES: [&str; 9] = [
    "no_match",
    "generic_analog",
    "atlas_ec",
    "aht22",
    "mcp_9808",
    "ring_temperature",
    "heater",
    "ds18b20",
    "k30_co2",
];

pub fn sensor_type_id_from_name(name: &str) -> u16 {
    for i in 0..SENSOR_NAMES.len() {
        if name == SENSOR_NAMES[i] {
            return u16::try_from(i).ok().unwrap();
        }
    }
    return 0;
}

pub fn sensor_name_from_type_id(id: usize) -> [u8; 16] {
    let mut rval = [b'\0'; 16];
    let name = SENSOR_NAMES[id].clone();
    rval[..name.len()].copy_from_slice(name.as_bytes());
    rval
}

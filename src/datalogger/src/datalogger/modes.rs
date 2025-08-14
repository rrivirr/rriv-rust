pub enum DataLoggerMode {
    Interactive,
    Field,
    HibernateUntil
}

pub enum DataLoggerSerialTxMode {
    Normal,
    Quiet,
    Watch
}


pub fn mode_text(mode: &DataLoggerMode) -> &'static str {
    match mode {
        DataLoggerMode::Interactive => "interactive",
        DataLoggerMode::Field => "field",
        DataLoggerMode::HibernateUntil => "hibernate",
    }
}

        // DataLoggerSerialTxMode::Watch => "watch",
        // DataLoggerSerialTxMode::Quiet => "quiet",
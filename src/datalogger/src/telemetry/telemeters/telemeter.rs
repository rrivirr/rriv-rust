// pub trait telemeter {
//     pub fn setup();
//     pub fn run_loop_iteration();
//     pub fn error() -> boolean; // maybe return an enum with errors
//     pub fn transmit(bytes: &[u8]);
// }


/*

datalogger.run_loop_iteration {

    telemeter.run_loop_iteration(); // checks if we are good, and if not does step(s) to cure
                                    // uses internal state of the driver


    if( ! telemeter.error() ) {
        telemeter.transmit(bytes)
    }

}

*/
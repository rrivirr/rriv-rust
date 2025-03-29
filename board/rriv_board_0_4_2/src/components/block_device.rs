use embedded_sdmmc::{Block, BlockDevice};
use rtt_target::rprintln;
use stm32f1xx_hal::usb::{Peripheral, UsbBus};
use usbd_storage::{subclass::{ufi::{Ufi, UfiCommand}, Command}, transport::bbb::BulkOnly};

use crate::{components::{Storage, STORAGE}, util::transform_u32_to_array_of_u8};

const BLOCK_SIZE: usize = 512;

static mut BLOCK_STATE: State = State {
    storage_offset: 0,
    sense_key: None,
    sense_key_code: None,
    sense_qualifier: None,
};

#[derive(Default)]
struct State {
    pub storage_offset: usize,
    pub sense_key: Option<u8>,
    pub sense_key_code: Option<u8>,
    pub sense_qualifier: Option<u8>,
}

impl State {
    pub fn reset(&mut self) {
        self.storage_offset = 0;
        self.sense_key = None;
        self.sense_key_code = None;
        self.sense_qualifier = None;
    }
}


pub fn process_ufi_command(
    mut command: Command<'_, UfiCommand, Ufi<BulkOnly<'_, UsbBus<Peripheral>, &mut [u8]>>>,
) {
    rprintln!("Command: {:?}", command.kind);

    match command.kind {
        UfiCommand::Inquiry { .. } => {
            // return device information
            // let result = command.try_write_data_all(&[
            //     0x00, 0b10000000, // removable media bit
            //     0,          // always zeros for UFI
            //     0x01,       // response data format always 1 for UFI
            //     0x1F,       // always 1F for UFI
            //     0, 0, 0, // reserved
            //     b'R', b'R', b'I', b'V', b' ', b' ', b' ', b' ', // vendor
            //     b'R', b'R', b'I', b'V', b' ', b' ', b' ', b' ', b' ', b' ', b' ', b' ', b' ',
            //     b' ', b' ', b' ', // product
            //     b'0', b'.', b'4', b'0', // todo: Firmware Version
            // ]);

            match command.try_write_data_all(&[
                0x00, 0b10000000, 0, 0x01, 0x1F, 0, 0, 0, b'F', b'o', b'o', b' ', b'B', b'a', b'r',
                b'0', b'F', b'o', b'o', b' ', b'B', b'a', b'r', b'0', b'F', b'o', b'o', b' ', b'B',
                b'a', b'r', b'0', b'1', b'.', b'2', b'3',
            ]){
                Ok(_) => {rprintln!("Inquiry response sent")},
                Err(err) => {rprintln!("error {:?}", err)},
            }
            command.pass();
        }
        UfiCommand::StartStop { .. }
        | UfiCommand::TestUnitReady
        | UfiCommand::PreventAllowMediumRemoval { .. } => {
            command.pass();
        }
        UfiCommand::ReadCapacity => {

            // let command_payload = [0x00, 0x00, 0x0b, 0x04, 0x00, 0x00, 0x02, 0x00]; // assume only 2 blocks

            // match command.try_write_data_all(&command_payload) {
            //     Ok(_) => {rprintln!("Success: Read capacity sent")},
            //     Err(err) => {rprintln!("error {:?}", err)},
            // }
            // command.pass();
            

            // return;

            unsafe {
                if STORAGE.is_none() {
                    rprintln!("Storage object is none in block device");
                    command.fail();
                    return;
                }
            }
            
            let storage: &mut Storage = unsafe { STORAGE.as_mut().unwrap() };
            let device = storage.volume_manager.device();
            let blocks: embedded_sdmmc::BlockCount =  match device.num_blocks(){
                Ok(num_blocks) => num_blocks,
                Err(err) => {
                    rprintln!("{:?}", err);
                    command.pass();
                    return
                },
            };

            rprintln!("{:?} blocks", blocks);
            let last_block_addr = transform_u32_to_array_of_u8(blocks.0 - 1);
            let block_size = 512_u32;
            let block_size = transform_u32_to_array_of_u8(block_size);
            let mut command_payload: [u8;8] = [0;8];
            command_payload[0..4].copy_from_slice(&last_block_addr);
            command_payload[4..8].copy_from_slice(&block_size);

            match command.try_write_data_all(&command_payload) {
                Ok(_) => {rprintln!("Block: Read capacity sent {:?}", command_payload)},
                Err(err) => {rprintln!("error {:?}", err)},
            }
            command.pass();
        }
        UfiCommand::RequestSense { .. } => unsafe {
            let result = command.try_write_data_all(&[
                0x70, // error code
                0x00,
                BLOCK_STATE.sense_key.unwrap_or(0),
                0x00,
                0x00,
                0x00,
                0x00,
                0x0A, // additional length
                0x00,
                0x00,
                0x00,
                0x00,
                BLOCK_STATE.sense_key_code.unwrap_or(0),
                BLOCK_STATE.sense_qualifier.unwrap_or(0),
                0x00,
                0x00,
                0x00,
                0x00,
            ]);
            BLOCK_STATE.reset();
            command.pass();
        },
        UfiCommand::ModeSense { .. } => {
            /* Read Only */
            // let _ =
            //     command.try_write_data_all(&[0x00, 0x46, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00]);

            /* Read Write */
            match command.try_write_data_all(&[0x00, 0x46, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]) {
                Ok(_) => {rprintln!("Block: mode sent")},
                Err(err) => {rprintln!("error {:?}", err)},
            }
            command.pass();
        }
        UfiCommand::Write { .. } => {
            command.pass();
        }
        UfiCommand::Read { lba, len } => unsafe {

            let storage: &mut Storage = unsafe { STORAGE.as_mut().unwrap() };

            let mut lba = lba as u32;
            let len = len as u32;

            for i in 0..len {
                let mut block: [Block; 1] = [Block::new(); 1];
                match storage.volume_manager.device().read(&mut block, embedded_sdmmc::BlockIdx(lba + i), "read") {
                    Ok(_) => {
                        match command.write_data(&block[0].contents) {
                            Ok(_) => {rprintln!("send block {:?}", block[0].contents)}
                            Err(err) => rprintln!("{:?}", err),
                        }; // TODO unwrap
                        // STATE.storage_offset += count;
                    },
                    Err(err) => rprintln!("{:?}", err),
                }
            }

            command.pass();




            // let lba = lba as u32;
            // let len = len as u32;
            // if STATE.storage_offset != len as usize * BLOCK_SIZE {
            //     loop {
            //         let count = command.write_data(&[0xF6; BLOCK_SIZE as usize]).unwrap(); // TODO unwrap;
            //         STATE.storage_offset += count;
            //         if count == 0 {
            //             break;
            //         }
            //     }
            // } else {
            //     command.pass();
            //     STATE.storage_offset = 0;
            // }

            // return;

            // if STATE.storage_offset != len as usize * BLOCK_SIZE {
            //     const DUMP_MAX_LBA: u32 = 0xCE;
            //     if lba < DUMP_MAX_LBA {
            //         /* requested data from dump */
            //         let start = (BLOCK_SIZE * lba as usize) + STATE.storage_offset;
            //         let end =
            //             (BLOCK_SIZE * lba as usize) + (BLOCK_SIZE as usize * len as usize);
            //         rprintln!("Data transfer >>>>>>>> [{}..{}]", start, end);
            //         // let count = command.write_data(&FAT[start..end]).unwrap(); // TODO unwrap
            //         // STATE.storage_offset += count;
            //     } else {
            //         /* fill with 0xF6 */
            //         loop {
            //             let count = command.write_data(&[0xF6; BLOCK_SIZE as usize]).unwrap(); // TODO unwrap;
            //             STATE.storage_offset += count;
            //             if count == 0 {
            //                 break;
            //             }
            //         }
            //     }
            // } else {
            //     command.pass();
            //     STATE.storage_offset = 0;
            // }
        },
        ref unknown_ufi_kind => {
            rprintln!("Unknown UFI command: {:?}", unknown_ufi_kind);
            unsafe {
                BLOCK_STATE.sense_key.replace(0x05); // illegal request
                BLOCK_STATE.sense_key_code.replace(0x20); // Invalid command operation
                BLOCK_STATE.sense_qualifier.replace(0x00); // Invalid command operation
            }
            command.fail();
        }
    }
}
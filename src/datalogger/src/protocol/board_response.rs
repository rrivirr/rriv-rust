// use serde_json::json;
// extern crate alloc;
// use crate::alloc::string::ToString;

// pub struct BoardResponse {
//     pub epoch: Option<i64>
// }

// impl BoardResponse {
//     pub fn new() -> BoardResponse {
//         BoardResponse {
//             epoch: None
//         }
//     }

//     pub fn render_json(&mut self) -> String {
//         let json = json!({
//             "epoch": self.epoch
//         });
//         let string = json.to_string();
//         return string;
//     }

// }
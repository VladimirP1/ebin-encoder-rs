use std::{
    fs::File,
    io::{self, BufRead, Read},
};

use ebin::{
    compress::{compress_block, decompress_block},
    quant::{QuantResult, State},
    quat::{Fix, Quat, RVec},
};

fn load_u8(path: String) -> Option<Vec<u8>> {
    let mut file = File::open(path).map(|x| Some(x)).unwrap_or(None)?;
    let mut buf = vec![];
    file.read_to_end(&mut buf)
        .map(|x| Some(x))
        .unwrap_or(None)?;
    Some(buf)
}

fn decode(buf: &[u8]) -> Option<()> {
    if !buf[..7].eq("EspLog0".as_bytes()) {
        return None;
    }

    let mut pos = 7;
    let mut accel_blk_size = 0;
    let mut accel_range = 0i32;
    let mut state = State::new();
    let mut tmp_quats = vec![];
    let mut prev_quat = Quat::default();
    let mut quats = vec![];
    let mut accels = vec![];

    while pos < buf.len() {
        match &buf[pos] {
            0x01 => {
                // gyro setup
                if buf[pos + 1] != 0x01 {
                    println!("unsupported algo revision");
                    return None;
                }
                tmp_quats.resize(
                    (buf[pos + 2] as u32 | ((buf[pos + 3] as u32) << 8)) as usize,
                    Quat::default(),
                );
                pos += 4;
                println!("gyro setup. block size: {}", tmp_quats.len());
            }
            0x02 => {
                // delta time
                let dt = (buf[pos + 1] as u32)
                    | ((buf[pos + 2] as u32) << 8)
                    | ((buf[pos + 3] as u32) << 16)
                    | ((buf[pos + 4] as u32) << 24);
                println!("delta time. dt: {}", dt);
                pos += 5;

                // generate timestamps
                let accel_div = quats.len() /  accels.len();
                for i in 0..quats.len() {

                }
            }
            0x03 => {
                // gyro data
                if let Some(res) = decompress_block(&state, &buf[pos + 1..], &mut tmp_quats) {
                    pos += res.bytes_eaten + 1;
                    quats.extend_from_slice(&tmp_quats);
                    state = res.new_state;
                    println!("gyro data block. len: {}", res.bytes_eaten);
                } else {
                    break;
                }
            }
            0x04 => {
                // accel setup
                accel_blk_size = buf[pos + 1] as usize;
                accel_range = 1 << buf[pos + 2];
                println!("accel setup. full_scale: {}", accel_range);
                pos += 3;
            }
            0x05 => {
                // accel data
                let mut apos = pos + 1;
                for _ in 0..accel_blk_size {
                    let ax = ((buf[apos + 0] as u16) + ((buf[apos + 1] as u16) << 8)) as i16;
                    let ay = ((buf[apos + 2] as u16) + ((buf[apos + 3] as u16) << 8)) as i16;
                    let az = ((buf[apos + 4] as u16) + ((buf[apos + 5] as u16) << 8)) as i16;
                    accels.push([ax, ay, az]);
                    apos += 6;
                }
                pos += 1 + 6 * accel_blk_size;
                println!("accel data. count: {}", accel_blk_size);
            }
            _ => {
                break;
            }
        }
    }
    dbg!(quats.len(), accels.len());
    Some(())
}

fn main() {
    let buf = load_u8("/home/user/Downloads/TEST.BIN".to_string()).expect("failed to open file");

    decode(&buf);
}

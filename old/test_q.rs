use std::{
    fs::File,
    io::{self, BufRead, Read, Write},
    slice,
};

use ebin::{
    compress::{compress_block, decompress_block},
    quant::{self, QuantResult, State},
    quat::{Fix, Quat, RVec},
};

fn parse_gcsv(path: String) -> Option<Vec<RVec>> {
    let tscale = 0.00180;
    let gscale = 0.00053263221;
    let file = File::open(path).map(|x| Some(x)).unwrap_or(None)?;
    let lines = io::BufReader::new(file).lines();
    Some(
        lines
            .skip_while(|x| {
                !x.as_ref()
                    .unwrap()
                    .chars()
                    .nth(0)
                    .unwrap_or('?')
                    .is_digit(10)
            })
            .map(|line| {
                line.unwrap()
                    .split(",")
                    .skip(1)
                    .take(3)
                    .map(|x| x.parse::<i32>().unwrap() as f32 * gscale * tscale)
                    .map(Fix::from_float)
                    .collect::<Vec<_>>()
            })
            .map(|v| RVec::new(v[0], v[1], v[2]))
            .collect(),
    )
}

fn parse_gcsv_q(path: String) -> Option<Vec<Quat>> {
    let rvs = parse_gcsv(path)?;
    let mut a = Quat::default();
    Some(
        rvs.iter()
            .map(|x| {
                a = a * Quat::from_rvec(x);
                a
            })
            .collect(),
    )
}

fn load_raw_q(path: String) -> Option<Vec<Quat>> {
    let mut file = File::open(path).map(|x| Some(x)).unwrap_or(None)?;
    let mut buf = vec![];
    file.read_to_end(&mut buf)
        .map(|x| Some(x))
        .unwrap_or(None)?;
    let quats = unsafe {
        std::slice::from_raw_parts(
            buf.as_ptr() as *const Quat,
            buf.len() / std::mem::size_of::<Quat>(),
        )
    };
    Some(Vec::from_iter(quats.iter().map(|x| x.to_owned())))
}

fn main() {
    // let quats = parse_gcsv_q("testdata/test.gcsv".to_string()).unwrap();
    let quats = load_raw_q("testdata/test.rawquat".to_string()).unwrap();

    let mut state = State::new();
    let mut data = vec![0; 8192 * 8192];
    let mut scratch = vec![0; 8192];
    let mut bytes_tot = 0;

    let mut file = File::create("quanted.bin").expect("failed to open file");

    for c in quats.chunks_exact(512) {
        // let res = compress_block(&state, &c, 14, &mut data[bytes_tot..], &mut scratch).unwrap();
        let res = state.quant_block(&c, 14, &mut scratch).unwrap();
        file.write_all(
            &unsafe { slice::from_raw_parts(scratch.as_ptr() as *const u8, scratch.len()) }
                [..res.bytes_put],
        )
        .expect("failed to write");
        state = res.new_state;
        bytes_tot += res.bytes_put;
    }
}

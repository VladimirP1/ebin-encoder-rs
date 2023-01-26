use std::{
    fs::File,
    io::{self, BufRead},
};

use ebin::{
    compress::{compress_block, decompress_block},
    quant::{QuantResult, State},
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

fn main() {
    let quats = parse_gcsv_q("testdata/test.gcsv".to_string()).unwrap();

    let mut state = State::new();
    let mut data = vec![0; 8192 * 8192];
    let mut scratch = vec![0; 8192];
    let mut bytes_tot = 0;

    for c in quats.chunks_exact(512) {
        let res = compress_block(&state, &c, 14, &mut data[bytes_tot..], &mut scratch).unwrap();
        state = res.new_state;
        bytes_tot += res.bytes_put;
    }

    dbg!(bytes_tot);

    let mut state = State::new();
    let mut quats_out = vec![Quat::default(); 8192 * 8192];
    let mut bytes_read = 0;
    let mut quats_put = 0;
    while bytes_read < bytes_tot {
        let res = decompress_block(
            &state,
            &data[bytes_read..],
            &mut quats_out[quats_put..quats_put + 512],
        )
        .unwrap();
        state = res.new_state;
        bytes_read += res.bytes_eaten;
        quats_put += res.quats_put;
    }

    let mut rmse = 0.0;
    for i in 0..quats_put {
        let err = (quats[i].conj() * quats_out[i]).to_rvec().norm().to_float() * 180.0 / 3.14;
        rmse += err * err;
        if err*err > 0.1 {
            dbg!(i);
        }
    }
    rmse = (rmse / (quats.len() as f32)).sqrt();
    
    dbg!(rmse);
    println!("{} quats in", quats_put);
    println!("{} bytes out", bytes_read);
}

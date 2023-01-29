use std::{
    fs::File,
    io::{self, BufRead, Read},
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

fn load_file(path: String) -> Option<Vec<u8>> {
    let mut file = File::open(path).map(|x| Some(x)).unwrap_or(None)?;
    dbg!(&file);
    let mut buf = vec![];
    file.read_to_end(&mut buf)
        .map(|x| Some(x))
        .unwrap_or(None)?;
    Some(Vec::from_iter(buf.iter().map(|x| x.to_owned())))
}

fn decompress(data: &[u8]) -> Vec<Quat> {
    let mut state = State::new();
    let mut quats_out = vec![Quat::default(); 8192 * 8192];
    let mut bytes_read = 0;
    let mut quats_put = 0;
    while bytes_read < data.len() {
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
    println!("{} quats in", quats_put);
    println!("{} bytes out", bytes_read);

    quats_out.resize(quats_put, Quat::default());
    quats_out
}

fn main() {
    let mut data = vec![0; 8192 * 8192];
    {
        // let quats = parse_gcsv_q("testdata/test.gcsv".to_string()).unwrap();
        let quats = load_raw_q("testdata/test.rawquat".to_string()).unwrap();
        let mut state = State::new();
        let mut scratch = vec![0; 8192];
        let mut bytes_tot = 0;
        let mut qbytes_tot = 0;
        
        for c in quats.chunks_exact(512) {
            let res = compress_block(&state, &c, 14, &mut data[bytes_tot..], &mut scratch).unwrap();
            state = res.new_state;
            bytes_tot += res.bytes_put;
            qbytes_tot += res.dbg_qbytes;
        }
        data.resize(bytes_tot, 0);

        dbg!(qbytes_tot);
        dbg!(bytes_tot);
    }

    let mut data2 =
        load_file("/home/user/src/src/ebin-encoder-cpp/build/compressed.bin".to_string()).unwrap();

    let quats = decompress(&data);
    let quats2 = decompress(&data2);

    let mut rmse = 0.0;
    for i in 0..quats.len() {
        if quats[i] != quats2[i] {
            dbg!(i);
        }
        let err = (quats[i].conj() * quats2[i]).to_rvec().norm().to_float() * 180.0 / 3.14;
        rmse += err * err;
        if err * err > 0.1 {
            dbg!(i);
        }
    }
    rmse = (rmse / (quats.len() as f32)).sqrt();

    dbg!(rmse);
}

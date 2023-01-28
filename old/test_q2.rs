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

fn quant_update(update: RVec, scale: u8, lim: i8) -> [i8; 3] {
    let r = [update.x.to_raw(), update.y.to_raw(), update.z.to_raw()];
    let u = r.map(|r| (r >> scale) as i8);

    let check_ovf = |orig: i32, quant: i8| {
        if quant as i32 == (orig >> scale) && quant.abs() <= lim {
            quant
        } else {
            if orig < 0 {
                -lim
            } else {
                lim
            }
        }
    };

    [
        check_ovf(r[0], u[0]),
        check_ovf(r[1], u[1]),
        check_ovf(r[2], u[2]),
    ]
}

fn dequant_update(update: [i8; 3], scale: u8) -> RVec {
    RVec::new(
        Fix::from_raw((update[0] as i32) << scale),
        Fix::from_raw((update[1] as i32) << scale),
        Fix::from_raw((update[2] as i32) << scale),
    )
}

fn is_saturated(v: [i8; 3], lim: i8) -> bool {
    v[0].abs() == lim || v[1].abs() == lim || v[2].abs() == lim
}

pub fn quant_block(state: &State, quats: &[Quat], qp: u8, out: &mut [i8]) -> Option<QuantResult> {
    let mut bytes_put = 0;
    let mut max_ang_err = Fix::from_i32(0);
    let mut new_state = state.clone();

    for i in 0..quats.len() {
        // compute angular acceleration update
        let q = quats[i];
        let q_update = new_state.q.conj() * q;
        let mut v_update = q_update.to_rvec() - new_state.v;

        println!(
            "D {} {} {} {} - {} {} {} {}",
            new_state.q.w.to_raw(),
            new_state.q.x.to_raw(),
            new_state.q.y.to_raw(),
            new_state.q.z.to_raw(),
            q.w.to_raw(),
            q.x.to_raw(),
            q.y.to_raw(),
            q.z.to_raw(),
        );

        // quantize update
        let mut sum = RVec::default();
        let mut correction_needed = true;
        while correction_needed {
            let update_quanted = quant_update(v_update, qp, 127);
            let update_dequanted = dequant_update(update_quanted, qp);

            println!(
                "{} {} {} {} -> {} {} {} -> {} {} {}",
                q_update.w.to_raw(),
                q_update.x.to_raw(),
                q_update.y.to_raw(),
                q_update.z.to_raw(),
                v_update.x.to_raw(),
                v_update.y.to_raw(),
                v_update.z.to_raw(),
                update_quanted[0],
                update_quanted[1],
                update_quanted[2],
            );

            sum = sum + update_dequanted;
            v_update = v_update - update_dequanted;

            correction_needed = is_saturated(update_quanted, 127);

            if bytes_put + 3 > out.len() {
                return None;
            }
            for b in 0..3 {
                out[bytes_put + b] = update_quanted[b];
            }
            bytes_put += 3;
        }

        println!("S {} {} {}", sum.x.to_raw(), sum.y.to_raw(), sum.z.to_raw());

        
        // update state
        new_state.v = new_state.v + sum;
        new_state.q = (new_state.q * Quat::from_rvec(&new_state.v)).normalize_safe();
        
        let cq = Quat::from_rvec(&new_state.v);
        println!(
            "C {} {} {} {}",
            cq.w.to_raw(),
            cq.x.to_raw(),
            cq.y.to_raw(),
            cq.z.to_raw()
        );
        // println!(
        //     "{} {} {} {}",
        //     new_state.q.w.to_raw(),
        //     new_state.q.x.to_raw(),
        //     new_state.q.y.to_raw(),
        //     new_state.q.z.to_raw()
        // );

        // update max quantization error
        max_ang_err = (new_state.q.conj() * q).to_rvec().norm().max(max_ang_err);
    }

    Some(QuantResult {
        new_state,
        bytes_put,
        max_ang_err,
    })
}

fn main() {
    // let quats = parse_gcsv_q("testdata/test.gcsv".to_string()).unwrap();
    let quats = load_raw_q("testdata/test.rawquat".to_string()).unwrap();

    // let mut pq = Quat::default();
    // for q in quats {
    //     let dq = pq.conj() * q;
    //     println!(
    //         "{} {} {} {}",
    //         dq.w.to_raw(),
    //         dq.x.to_raw(),
    //         dq.y.to_raw(),
    //         dq.z.to_raw()
    //     );
    //     pq = q;
    // }
    let mut state = State::new();
    let mut out = vec![0i8; 1024 * 1024];
    let res = quant_block(&state, &quats[..8192], 14, &mut out);
}

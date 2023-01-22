use ebin::{
    coder::State,
    quat::{Fix, Quat, RVec},
};

fn main() {
    let mut buf = vec![0u8; 100000 * 6];
    let mut quats = Vec::new();
    {
        let mut state = State::new();
        let mut cur_q = Quat::default();
        for _ in 0..100000 {
            quats.push(cur_q);
            cur_q = cur_q
                * Quat::from_rvec(&RVec::new(
                    Fix::from_float(0.02),
                    Fix::from_float(0.01),
                    Fix::from_float(0.001),
                ));
        }

        let res = state.quant_block(&quats, 14, &mut buf).unwrap();

        dbg!(res.bytes_put);
        dbg!(res.max_ang_err.to_float() * 180.0 / 3.14);
    }
    let mut quats2 = vec![Quat::default(); 200000];
    {
        let mut state = State::new();
        let res = state.dequant_block(&buf, 14, &mut quats2).unwrap();
    }

    for i in 0..10000 {
        println!("{}", (quats[i].to_rvec().norm() - quats2[i].to_rvec().norm()).to_float() * 180.0/3.14);
    }
    // for i in 0..res.bytes_put {
    //     println!("{}", buf[i]);
    // }
}

use ebin::{
    quant::{QuantResult, State},
    quat::{Fix, Quat, RVec},
    rans::{rans_encode, LaplaceCdf, rans_decode}, compress::{compress_block, decompress_block},
};

fn main() {
    let mut buf = vec![0i8; 2000];
    let mut quats = Vec::new();
    let res: QuantResult;
    {
        let mut state = State::new();
        let mut cur_q = Quat::default();
        for _ in 0..256 {
            quats.push(cur_q);
            cur_q = cur_q
                * Quat::from_rvec(&RVec::new(
                    Fix::from_float(0.02),
                    Fix::from_float(0.01),
                    Fix::from_float(0.0),
                ));
        }

        res = state.quant_block(&quats, 14, &mut buf).unwrap();

        dbg!(res.bytes_put);
        dbg!(res.max_ang_err.to_float() * 180.0 / 3.14);
    }
    // let mut quats2 = vec![Quat::default(); 256];
    // {
    //     let mut state = State::new();
    //     let res = state
    //         .dequant_block(&buf[0..res.bytes_put], 14, &mut quats2)
    //         .unwrap();
    //     dbg!(res.quats_put);
    // }

    let mut state = State::new();
    let mut data = vec![0; 8192];
    let mut scratch = vec![0; 8192];

    let res = compress_block(&state, &quats, 14, &mut data, &mut scratch).unwrap();
    dbg!(res.bytes_put);
    let mut state = State::new();
    let mut quats3 = vec![Quat::default(); 256];
    let res1 = decompress_block(&state, &data[0..119], &mut quats3).unwrap();
    dbg!(res1);


    let mut rmse = 0.0;
    for i in 0..quats.len() {
        let err = (quats[i].conj() * quats3[i]).to_rvec().norm().to_float() * 180.0 / 3.14;
        rmse += err * err;
    }
    rmse = (rmse / (quats.len() as f32)).sqrt();
    dbg!(rmse);
}

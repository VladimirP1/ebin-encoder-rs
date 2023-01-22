use ebin::{
    coder::State,
    quat::{Fix, Quat, RVec},
};

fn main() {
    let mut state = State::new();
    let mut cur_q = Quat::default();
    let mut quats = Vec::new();
    for _ in 0..256 * 10 {
        quats.push(cur_q);
        // println!("{}", cur_q.to_rvec().norm());
        cur_q = cur_q
            * Quat::from_rvec(&RVec::new(
                Fix::from_float(0.01),
                Fix::from_float(0.01),
                Fix::from_float(0.01),
            ));
    }

    let mut buf = vec![0u8; 20000];
    let res = state.quant_block(&quats, 14, &mut buf).unwrap();
    // coder.accept(&res);
    // let res = coder.quant_block(&quats[256..512], &mut buf, 14).unwrap();
    dbg!(res.bytes_put);
    dbg!(res.max_ang_err.to_float() * 180.0 / 3.14);

    for i in 0..res.bytes_put {
        println!("{}", buf[i]);
    }
}

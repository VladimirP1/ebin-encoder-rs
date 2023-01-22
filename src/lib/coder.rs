use crate::quat::{Fix, Quat, RVec};

#[derive(Copy, Clone, Debug)]
pub struct State {
    q: Quat, // decoder's quat
    v: RVec, // decoder's angular velocity
}

#[derive(Copy, Clone, Debug)]
pub struct QuantResult {
    pub new_state: State,
    pub bytes_put: usize,
    pub max_ang_err: Fix,
}

impl State {
    pub fn new() -> State {
        State {
            q: Quat::default(),
            v: RVec::default(),
        }
    }

    pub fn quant_block(self, quats: &[Quat], qp: u8, out: &mut [u8]) -> Option<QuantResult> {
        let mut bytes_put = 0;
        let mut max_ang_err = Fix::from_i32(0);
        let mut new_state = self;

        for i in 0..quats.len() {
            // compute angular acceleration update
            let q = quats[i];
            let q_update = new_state.q.conj() * q;
            let mut v_update = q_update.to_rvec() - new_state.v;

            // quantize update
            let mut sum = RVec::default();
            let mut correction_needed = true;
            while correction_needed {
                let update_quanted = quant_update(v_update, qp, 127);
                let update_dequanted = dequant_update(update_quanted, qp);

                sum = sum + update_dequanted;
                v_update = v_update - update_dequanted;

                correction_needed = is_saturated(update_quanted, 127);

                if bytes_put + 3 > out.len() {
                    return None;
                }
                for b in 0..3 {
                    out[bytes_put + b] = update_quanted[b] as u8;
                }
                bytes_put += 3;
            }
            
            // update state
            new_state.v = new_state.v + sum;
            new_state.q = (new_state.q * Quat::from_rvec(&new_state.v)).normalize_safe();

            // update max quantization error
            max_ang_err = (new_state.q.conj() * q).to_rvec().norm().max(max_ang_err);
        }

        Some(QuantResult {
            new_state,
            bytes_put,
            max_ang_err,
        })
    }
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

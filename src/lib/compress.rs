use crate::{quant::State, quat::Quat};

#[derive(Copy, Clone, Debug)]
pub struct CompressResult {
    pub new_state: State,
    pub bytes_put: usize,
}

const VAR_TABLE: [f64; 16] = [
    0.015625, 0.03125, 0.0625, 0.125, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0,
    256.0, 512.0,
];
const SCALE: i32 = 15;

fn to_i8(a: &[u8]) -> &[i8] {
    unsafe { &*(a as *const _ as *const [i8]) }
}

fn to_u8(a: &[i8]) -> &[u8] {
    unsafe { &*(a as *const _ as *const [u8]) }
}

pub fn compress_block(
    state: &State,
    quats: &[Quat],
    qp: u8,
    data: &mut [u8],
    scratch: &mut [i8],
) -> Option<CompressResult> {
    let quant_result = state.quant_block(quats, qp, scratch)?;
    let var = (scratch[0..quant_result.bytes_put]
        .iter()
        .map(|&x| ((x as i64) * (x as i64)) as i64)
        .sum::<i64>() as f64)
        / (quant_result.bytes_put as f64);
    let i_var = VAR_TABLE
        .partition_point(|&x| x < var)
        .min(VAR_TABLE.len() - 1);
    let i_var = 1;
    let var = VAR_TABLE[i_var];
    let mdl = LaplaceCdf::new(var, SCALE);
    let rans_result = rans_encode(&scratch[..quant_result.bytes_put], &mut data[2..], &mdl)?;
    let cksum = scratch[0..quant_result.bytes_put]
        .iter()
        .map(|x| x.to_owned() as u8)
        .reduce(|a, b| a.wrapping_add(b))
        .unwrap_or(0);
    data[0] = qp;
    data[1] = i_var as u8 | (cksum << 5);

    dbg!(i_var);

    Some(CompressResult {
        new_state: quant_result.new_state,
        bytes_put: rans_result + 2,
    })
}

pub fn decompress_block(state: &State, data: &[u8], quats: &mut [Quat]) -> Option<usize> {
    let qp = data[0];
    let i_var = (data[1] & 0x1f) as usize;
    let cksum = data[1] >> 5;
    dbg!(qp, i_var, cksum);

    let mdl = LaplaceCdf::new(VAR_TABLE[i_var], SCALE);

    let mut rstate = ((data[2] as u32) << 0)
        | ((data[3] as u32) << 8)
        | ((data[4] as u32) << 16)
        | ((data[5] as u32) << 24);
    let mut bytes_eaten = 6;

    let mut quats_put = 0;
    let mut new_state = state.clone();
    let mut own_cksum = 0;
    let mask = (1 << mdl.scale()) - 1;
    while quats_put < quats.len() {
        let mut s = [0, 0, 0];
        for i in 0..3 {
            let cum = rstate & mask;
            let sym = mdl.icdf(cum);
            s[i] = sym as i8;
            own_cksum = (s[i] as u8).wrapping_add(own_cksum);

            let start = mdl.cdf(sym);
            let freq = mdl.cdf(sym + 1) - start;

            rstate = freq * (rstate >> mdl.scale()) + (rstate & mask) - start;

            while rstate < RANS_BYTE_L {
                if bytes_eaten >= data.len() {
                    dbg!(bytes_eaten, quats_put);
                    panic!();
                    return None;
                }
                rstate = (rstate << 8) | data[bytes_eaten] as u32;
                bytes_eaten += 1;
            }
        }

        if let Some(q) = new_state.dequant_one(&s, qp) {
            if quats_put >= quats.len() {
                dbg!(bytes_eaten, quats_put);
                panic!();
                return None;
            }
            quats[quats_put] = q;
            quats_put += 1;
        }
    }
    dbg!(own_cksum&0x07, cksum);
    if (own_cksum & 0x07) != cksum {
        return None
    }
    Some(bytes_eaten)
}

pub fn rans_encode<T: Cdf>(data: &[i8], out: &mut [u8], mdl: &T) -> Option<usize> {
    let mut state = RANS_BYTE_L;
    let mut bytes_put = 0;
    let mut cntn = 0;
    for sym in data.iter().rev() {
        let start = mdl.cdf(*sym as i32);
        let freq = mdl.cdf(*sym as i32 + 1) - start;
        let x_max = ((RANS_BYTE_L >> mdl.scale()) << 8) * freq;
        let mut cnt = 0;
        while state >= x_max {
            cnt += 1;
            if bytes_put >= out.len() {
                return None;
            }
            out[bytes_put] = (state & 0xff) as u8;
            bytes_put += 1;
            state >>= 8;
        }
        state = ((state / freq) << mdl.scale()) + (state % freq) + start;
        cntn += 1;
    }
    out[bytes_put + 0] = (state >> 24) as u8;
    out[bytes_put + 1] = (state >> 16) as u8;
    out[bytes_put + 2] = (state >> 8) as u8;
    out[bytes_put + 3] = (state >> 0) as u8;
    bytes_put += 4;
    out[0..bytes_put].reverse();
    Some(bytes_put)
}

const RANS_BYTE_L: u32 = 1 << 23;

pub trait Cdf {
    fn cdf(&self, x: i32) -> u32;
    fn icdf(&self, y: u32) -> i32;
    fn scale(&self) -> i32;
}

#[derive(Copy, Clone)]
pub struct LaplaceCdf {
    var: f64,
    b: f64,
    scale: i32,
}

impl LaplaceCdf {
    pub fn new(var: f64, scale: i32) -> LaplaceCdf {
        LaplaceCdf {
            var,
            b: (var / 2.0).sqrt(),
            scale,
        }
    }
}

impl Cdf for LaplaceCdf {
    fn cdf(&self, x: i32) -> u32 {
        if x <= -128 {
            return 0;
        }
        if x > 128 {
            return 1 << self.scale;
        }

        let xs = x as f64 - 0.5;
        let cum = if xs < 0.0 {
            (xs / self.b).exp() / 2.0
        } else {
            1.0 - (-xs / self.b).exp() / 2.0
        };

        (cum * (((1 << self.scale) as f64) - 257.0)) as u32 + (x + 128) as u32
    }

    fn icdf(&self, y: u32) -> i32 {
        let mut l = -129;
        let mut r = 129;
        while l + 1 != r {
            let mid = (l + r) / 2;
            if self.cdf(mid) <= y && self.cdf(mid + 1) > y {
                return mid;
            }
            if self.cdf(mid) <= y {
                l = mid;
            } else {
                r = mid;
            }
        }
        r
    }

    fn scale(&self) -> i32 {
        self.scale
    }
}

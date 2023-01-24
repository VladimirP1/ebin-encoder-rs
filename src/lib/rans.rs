const RANS_BYTE_L: u32 = 1 << 23;

pub fn rans_encode<T: Cdf>(data: &[i8], out: &mut [u8], mdl: &T) -> Option<usize> {
    let mut state = RANS_BYTE_L;
    let mut bytes_put = 0;
    for sym in data.iter().rev() {
        let start = mdl.cdf(*sym as i32);
        let freq = mdl.cdf(*sym as i32 + 1) - start;
        let x_max = ((RANS_BYTE_L >> mdl.scale()) << 8) * freq;
        while state >= x_max {
            if bytes_put >= out.len() {
                return None;
            }
            out[bytes_put] = (state & 0xff) as u8;
            bytes_put += 1;
            state >>= 8;
        }
        state = ((state / freq) << mdl.scale()) + (state % freq) + start;
    }
    out[bytes_put + 0] = (state >> 24) as u8;
    out[bytes_put + 1] = (state >> 16) as u8;
    out[bytes_put + 2] = (state >> 8) as u8;
    out[bytes_put + 3] = (state >> 0) as u8;
    bytes_put += 4;
    out[0..bytes_put].reverse();
    Some(bytes_put)
}

pub fn rans_decode<T: Cdf>(data: &[u8], out: &mut [i8], mdl: &T) -> Option<usize> {
    let mut state = ((data[0] as u32) << 0)
        | ((data[1] as u32) << 8)
        | ((data[2] as u32) << 16)
        | ((data[3] as u32) << 24);
    let mut bytes_eaten = 4;

    let mask = (1 << mdl.scale()) - 1;
    for sym in out {
        let cum = state & mask;
        *sym = mdl.icdf(cum) as i8;
        let start = mdl.cdf(*sym as i32);
        let freq = mdl.cdf(*sym as i32 + 1) - start;

        state = freq * (state >> mdl.scale()) + (state & mask) - start;

        while state < RANS_BYTE_L {
            if bytes_eaten >= data.len() {
                return None;
            }
            state = (state << 8) | data[bytes_eaten] as u32;
            bytes_eaten += 1;
        }
    }
    Some(bytes_eaten)
}

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

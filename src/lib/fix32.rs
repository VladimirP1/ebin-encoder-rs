use std::fmt::{Debug, Display};
use std::ops::{Add, Div, Mul, Neg, Sub};

#[derive(Copy, Clone, PartialOrd, PartialEq, Ord, Eq)]
pub struct Fix32<const N: usize> {
    v: i32,
}

impl<const N: usize> Fix32<N> {
    pub const MULT: i32 = 1i32 << N;

    pub const E: Fix32<N> = Fix32 {
        v: (6267931151224907085i64 >> (61 - N)) as i32,
    };
    pub const PI: Fix32<N> = Fix32 {
        v: (7244019458077122842i64 >> (61 - N)) as i32,
    };
    pub const HALF_PI: Fix32<N> = Fix32 {
        v: (7244019458077122842i64 >> (62 - N)) as i32,
    };
    pub const TWO_PI: Fix32<N> = Fix32 {
        v: (7244019458077122842i64 >> (60 - N)) as i32,
    };

    pub fn from_raw(x: i32) -> Fix32<N> {
        Fix32 { v: x }
    }

    pub fn from_i32(x: i32) -> Fix32<N> {
        Fix32 { v: x * Self::MULT }
    }

    pub fn to_raw(&self) -> i32 {
        self.v
    }

    pub fn from_float(x: f32) -> Fix32<N> {
        Fix32 {
            v: (if x >= 0.0 {
                x * (Self::MULT as f32) + 0.5
            } else {
                x * (Self::MULT as f32) - 0.5
            }) as i32,
        }
    }

    pub fn to_float(&self) -> f32 {
        (self.v as f64 / Self::MULT as f64) as f32
    }

    pub fn atan2(&self, x: Fix32<N>) -> Fix32<N> {
        fn atan_s<const N: usize>(x: Fix32<N>) -> Fix32<N> {
            debug_assert!(x.v >= 0 && x.v <= Fix32::<N>::MULT);
            let fa: Fix32<N> = Fix32 {
                v: (716203666280654660i64 >> (63 - N)) as i32,
            };
            let fb: Fix32<N> = Fix32 {
                v: (-2651115102768076601i64 >> (63 - N)) as i32,
            };
            let fc: Fix32<N> = Fix32 {
                v: (9178930894564541004i64 >> (63 - N)) as i32,
            };
            let xx = x * x;
            return ((fa * xx + fb) * xx + fc) * x;
        }
        fn atan_div<const N: usize>(y: Fix32<N>, x: Fix32<N>) -> Fix32<N> {
            debug_assert!(x.v != 0);
            if y.v < 0 {
                if x.v < 0 {
                    atan_div(-y, -x)
                } else {
                    atan_div(-y, x)
                }
            } else if x.v < 0 {
                -atan_div(y, -x)
            } else {
                debug_assert!(y.v >= 0);
                debug_assert!(x.v > 0);
                if y.v > x.v {
                    Fix32::<N>::HALF_PI - atan_s(x / y)
                } else {
                    atan_s(y / x)
                }
            }
        }
        if x.v == 0 {
            debug_assert!(self.v != 0);
            if self.v > 0 {
                Self::HALF_PI
            } else {
                -Self::HALF_PI
            }
        } else {
            let ret = atan_div(*self, x);
            if x.v < 0 {
                if self.v >= 0 {
                    ret + Self::PI
                } else {
                    ret - Self::PI
                }
            } else {
                ret
            }
        }
    }

    pub fn sin(&self) -> Fix32<N> {
        let x = self.fmod(Self::TWO_PI);
        let x = x / Self::HALF_PI;
        let x = if x.v < 0 { x + Self::from_i32(4) } else { x };
        let (x, sign) = if x > Self::from_i32(2) {
            (x - Self::from_i32(2), -1)
        } else {
            (x, 1)
        };
        let x = if x > Self::from_i32(1) {
            Self::from_i32(2) - x
        } else {
            x
        };
        let x2 = x * x;
        Self::from_i32(sign)
            * x
            * (Self::PI
                - x2 * (Self::TWO_PI - Self::from_i32(5) - x2 * (Self::PI - Self::from_i32(3))))
            / Self::from_i32(2)
    }

    pub fn cos(&self) -> Fix32<N> {
        (Self::HALF_PI + *self).sin()
    }

    pub fn sqrt(&self) -> Fix32<N> {
        debug_assert!(self.v >= 0);
        if self.v == 0 {
            return *self;
        }

        let mut num: i64 = (self.v as i64) << N;
        let mut res: i64 = 0;
        let mut bit = 1 << ((find_highest_bit(self.v) + (N as u32)) / 2 * 2);

        while bit != 0 {
            let val = res + bit;
            res >>= 1;
            if num >= val {
                num -= val;
                res += bit;
            }
            bit >>= 2;
        }

        if num > res {
            res += 1;
        }

        fn find_highest_bit(x: i32) -> u32 {
            let mut x = x;
            let mut r = 0;
            loop {
                x >>= 1;
                if x == 0 {
                    break;
                }
                r += 1;
            }
            r
        }
        Fix32 { v: res as i32 }
    }

    pub fn fmod(&self, m: Fix32<N>) -> Fix32<N> {
        Fix32 { v: self.v % m.v }
    }
}

impl<const N: usize> Add for Fix32<N> {
    type Output = Fix32<N>;

    fn add(self, rhs: Self) -> Self::Output {
        Fix32 { v: self.v + rhs.v }
    }
}

impl<const N: usize> Sub for Fix32<N> {
    type Output = Fix32<N>;

    fn sub(self, rhs: Self) -> Self::Output {
        Fix32 { v: self.v - rhs.v }
    }
}

impl<const N: usize> Mul for Fix32<N> {
    type Output = Fix32<N>;

    fn mul(self, rhs: Self) -> Self::Output {
        let val = (self.v as i64) * (rhs.v as i64) / (Self::MULT as i64 / 2);
        Fix32 {
            v: ((val / 2) + (val % 2)) as i32,
        }
    }
}

impl<const N: usize> Div for Fix32<N> {
    type Output = Fix32<N>;

    fn div(self, rhs: Self) -> Self::Output {
        let val = (self.v as i64) * (Self::MULT as i64) * 2 / (rhs.v as i64);
        Fix32 {
            v: ((val / 2) + (val % 2)) as i32,
        }
    }
}

impl<const N: usize> Neg for Fix32<N> {
    type Output = Fix32<N>;

    fn neg(self) -> Self::Output {
        Fix32 { v: -self.v }
    }
}

impl<const N: usize> Debug for Fix32<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Fix32")
            .field("v", &self.to_float())
            .finish()
    }
}

impl<const N: usize> Display for Fix32<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(format!("{}", self.to_float()).as_str())?;
        Ok(())
    }
}

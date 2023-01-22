use std::ops::{Add, Mul, Neg, Sub};

use crate::fix32::Fix32;

pub type Fix = Fix32<27>;
pub type RVec = Vec3<27>;

#[derive(Copy, Clone, Debug)]
pub struct Quat {
    w: Fix,
    x: Fix,
    y: Fix,
    z: Fix,
}

impl Default for Quat {
    fn default() -> Self {
        Self {
            w: Fix::from_i32(1),
            x: Fix::from_i32(0),
            y: Fix::from_i32(0),
            z: Fix::from_i32(0),
        }
    }
}

impl Quat {
    pub fn new(w: Fix, x: Fix, y: Fix, z: Fix) -> Quat {
        Quat { w, x, y, z }
    }

    pub fn from_rvec(v: &RVec) -> Quat {
        let theta2 = v.x * v.x + v.y * v.y + v.z * v.z;
        if theta2.to_raw() > 16 {
            let theta = theta2.sqrt();
            let half_theta = theta * Fix::from_float(0.5);
            let k = half_theta.sin() / theta;
            Quat {
                w: half_theta.cos(),
                x: v.x * k,
                y: v.y * k,
                z: v.z * k,
            }
        } else {
            let k = Fix::from_float(0.5);
            Quat {
                w: Fix::from_i32(1),
                x: v.x * k,
                y: v.y * k,
                z: v.z * k,
            }
        }
    }

    pub fn to_rvec(&self) -> RVec {
        let sin_theta2 = self.x * self.x + self.y * self.y + self.z * self.z;
        if sin_theta2.to_raw() <= 0 {
            return RVec {
                x: self.x * Fix::from_i32(2),
                y: self.y * Fix::from_i32(2),
                z: self.z * Fix::from_i32(2),
            };
        }

        let sin_theta = sin_theta2.sqrt();
        let cos_theta = self.w;
        let two_theta = Fix::from_i32(2)
            * if cos_theta.to_raw() < 0 {
                (-sin_theta).atan2(-cos_theta)
            } else {
                sin_theta.atan2(cos_theta)
            };
        let k = two_theta / sin_theta;
        RVec {
            x: self.x * k,
            y: self.y * k,
            z: self.z * k,
        }
    }

    pub fn conj(&self) -> Quat {
        Quat {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn norm(&self) -> Fix {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(&mut self) -> Quat {
        let norm = self.norm();
        self.sdiv(norm)
    }

    pub fn normalize_safe(&mut self) -> Quat {
        let norm = self.norm();
        if norm.to_raw() == 0 {
            return Quat {
                w: Fix::from_raw(0),
                x: Fix::from_raw(0),
                y: Fix::from_raw(0),
                z: Fix::from_raw(0),
            };
        }
        self.sdiv(norm)
    }

    pub fn rotate_point(&self, p: &RVec) -> RVec {
        let qq = (*self)
            * Quat {
                w: Fix::from_raw(0),
                x: p.x,
                y: p.y,
                z: p.z,
            }
            * self.conj();
        RVec {
            x: qq.x,
            y: qq.y,
            z: qq.z,
        }
    }

    pub fn smul(&self, x: Fix) -> Quat {
        Quat {
            w: self.w * x,
            x: self.x * x,
            y: self.y * x,
            z: self.z * x,
        }
    }

    pub fn sdiv(&self, x: Fix) -> Quat {
        Quat {
            w: self.w / x,
            x: self.x / x,
            y: self.y / x,
            z: self.z / x,
        }
    }
}

impl Add for &Quat {
    type Output = Quat;

    fn add(self, rhs: Self) -> Self::Output {
        Quat {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Mul for &Quat {
    type Output = Quat;

    fn mul(self, rhs: Self) -> Self::Output {
        Quat {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

impl Add for Quat {
    type Output = Quat;

    fn add(self, rhs: Self) -> Self::Output {
        &self + &rhs
    }
}

impl Mul for Quat {
    type Output = Quat;

    fn mul(self, rhs: Self) -> Self::Output {
        &self * &rhs
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Vec3<const N: usize> {
    pub x: Fix32<N>,
    pub y: Fix32<N>,
    pub z: Fix32<N>,
}

impl<const N: usize> Vec3<N> {
    pub fn new(x: Fix32<N>, y: Fix32<N>, z: Fix32<N>) -> Vec3<N> {
        Vec3 { x, y, z }
    }

    pub fn smul(&self, k: Fix32<N>) -> Vec3<N> {
        Vec3 {
            x: self.x * k,
            y: self.y * k,
            z: self.z * k,
        }
    }

    pub fn sdiv(&self, k: Fix32<N>) -> Vec3<N> {
        Vec3 {
            x: self.x / k,
            y: self.y / k,
            z: self.z / k,
        }
    }

    pub fn norm(&self) -> Fix32<N> {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalized(&self) -> Vec3<N> {
        self.sdiv(self.norm())
    }
}

impl<const N: usize> Add for Vec3<N> {
    type Output = Vec3<N>;

    fn add(self, rhs: Self) -> Self::Output {
        Vec3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl<const N: usize> Sub for Vec3<N> {
    type Output = Vec3<N>;

    fn sub(self, rhs: Self) -> Self::Output {
        Vec3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl<const N: usize> Neg for Vec3<N> {
    type Output = Vec3<N>;

    fn neg(self) -> Self::Output {
        Vec3 {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl<const N: usize> Default for Vec3<N> {
    fn default() -> Self {
        Self {
            x: Fix32::<N>::from_i32(0),
            y: Fix32::<N>::from_i32(0),
            z: Fix32::<N>::from_i32(0),
        }
    }
}

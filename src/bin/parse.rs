use std::{
    fs::File,
    io::{self, BufRead},
};

use ebin::quat::{Fix, Quat, RVec};

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

fn main() {
    let q = parse_gcsv_q("testdata/test.gcsv".to_string());
    dbg!(q);
}

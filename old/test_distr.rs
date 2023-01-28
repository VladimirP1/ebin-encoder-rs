use std::{
    fs::File,
    io::{self, BufRead, Read},
};

use ebin::{
    compress::{compress_block, decompress_block, LaplaceCdf, Cdf},
    quant::{QuantResult, State},
    quat::{Fix, Quat, RVec},
};

fn main() {
    let mdl = LaplaceCdf::new(0.0625, 15);
    for i in -128..129 {
        println!("{}", mdl.cdf(i));
    }
}

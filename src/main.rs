#![no_main]
#![no_std]

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use cortex_m_rt::entry;
use lsm303agr::{Lsm303agr, MagMode, MagOutputDataRate};
use microbit::hal::pac::twim0::frequency::FREQUENCY_A;
use microbit::hal::time::U32Ext;
use microbit::hal::{Delay, Rng, Twim};
use microbit::pac::PWM0;
use microbit::Board;
use panic_halt as _;
use rtt_target::{rprint, rprintln, rtt_init_print};

// 2962^255 ≈ 1.7955141e+885
const MAX_PLACES: usize = 886;
const MAX_U16_PLACES: usize = 5;

const SAF_PRI_COU: usize = 49;
static SAF_PRIMES: [u16; SAF_PRI_COU] = [
    5, 7, 11, 23, 47, 59, 83, 107, 167, 179, 227, 263, 347, 359, 383, 467, 479, 503, 563, 587, 719,
    839, 863, 887, 983, 1019, 1187, 1283, 1307, 1319, 1367, 1439, 1487, 1523, 1619, 1823, 1907,
    2027, 2039, 2063, 2099, 2207, 2447, 2459, 2579, 2819, 2879, 2903, 2963,
];

use crate::CheckCondition::{GeneratorGreaterOrEqualToPrime, GeneratorLessThenTwo, Ok};

const MAX_DUTY: u16 = 512 as u16;
const MAX_AMOUNT: u16 = 1;
const HALF_AMOUNT: u16 = MAX_DUTY / 2;

use microbit::hal::gpio::Level;

use microbit::hal::pwm::{
    Channel::{self, C0 as CHRED, C1 as CHGREEN, C2 as CHBLUE},
    CounterMode, Prescaler, Pwm,
};

#[entry]
fn entry() -> ! {
    rtt_init_print!();

    let b = Board::take().unwrap();
    let mut rng = Rng::new(b.RNG);
    let mut del = Delay::new(b.SYST);

    let i2c = Twim::new(b.TWIM0, b.i2c_internal.into(), FREQUENCY_A::K100);
    let mut e_compass = Lsm303agr::new_with_i2c(i2c);
    e_compass.init().unwrap();

    e_compass
        .set_mag_mode_and_odr(&mut del, MagMode::LowPower, MagOutputDataRate::Hz10)
        .unwrap();

    let mag_con = e_compass.into_mag_continuous();

    let mut mag_con = match mag_con {
        core::result::Result::Ok(mc) => mc,
        _ => panic!(),
    };

    let pwm0 = Pwm::new(b.PWM0);

    pwm0.set_counter_mode(CounterMode::Up)
        .set_prescaler(Prescaler::Div4)
        .set_period(7810.hz());

    let pins = b.pins;
    pwm0.set_output_pin(
        CHRED,
        pins.p0_02.into_push_pull_output(Level::Low).degrade(),
    )
    .set_output_pin(
        CHGREEN,
        pins.p0_03.into_push_pull_output(Level::Low).degrade(),
    )
    .set_output_pin(
        CHBLUE,
        pins.p0_04.into_push_pull_output(Level::Low).degrade(),
    );

    let mut prime_ix = 0;
    loop {
        set_duty(&pwm0, CHRED, MAX_AMOUNT);
        set_duty(&pwm0, CHBLUE, MAX_AMOUNT);
        set_duty(&pwm0, CHGREEN, MAX_AMOUNT);
        del.delay_ms(250u32);

        let mag_sta = mag_con.mag_status();

        if mag_sta.is_err() {
            continue;
        }

        let mag_sta = unsafe { mag_sta.unwrap_unchecked() };

        if !mag_sta.xyz_new_data() {
            continue;
        }

        let mag_fie = mag_con.magnetic_field();
        if mag_fie.is_err() {
            continue;
        }

        let mag_fie = unsafe { mag_fie.unwrap_unchecked() };

        let uns = mag_fie.xyz_unscaled();

        rprintln!("uns.0 {}", uns.0);
        rprintln!("uns.1 {}", uns.1);
        rprintln!("uns.2 {}", uns.2);

        fn sq(num: i16) -> i32 {
            (num as i32).pow(2)
        }

        let nt = mag_fie.xyz_nt();

        rprintln!("\nnt.0 {}", nt.0);
        rprintln!("nt.1 {}", nt.1);
        rprintln!("nt.2 {}", nt.2);

        let raw = mag_fie.xyz_raw();

        rprintln!("\nraw.0 {}", raw.0);
        rprintln!("raw.1 {}", raw.1);
        rprintln!("raw.2 {}\n", raw.2);

        let x_sq = sq(uns.0);
        let y_sq = sq(uns.1);
        let z_sq = sq(uns.2);

        let vec_mag = herons_sqrt((x_sq + y_sq + z_sq) as u16);
        rprintln!("vec_mag {}", vec_mag);

        let sp = SAF_PRIMES[prime_ix];
        prime_ix = ((prime_ix as i32 + 1) % SAF_PRI_COU as i32) as usize;

        rprintln!("sp {}\n", sp);

        let vec_mag = vec_mag as u16;

        let ck = cond_ck(vec_mag, sp);
        let (d_amount, r_amount, g_amount, b_amount) = if ck == CheckCondition::Ok {
            let sp = to_decimals(sp);
            let sp = sp.as_slice();

            let gen = to_decimals(vec_mag);
            let gen = gen.as_slice();

            rprint!("ali private generation| ");
            let (alice_sec, alice_rem) = private_generation(&mut rng, &gen, &sp);
            rprintln!("sec {}, rem {}.", alice_sec, alice_rem);

            rprint!("bob private generation| ");
            let (bob_sec, bob_rem) = private_generation(&mut rng, &gen, &sp);
            rprintln!("sec {}, rem {}.", bob_sec, bob_rem);

            rprint!("ali shared generation| ");
            let akey = shared_generation(bob_rem, alice_sec, &sp);
            rprintln!("key {}.", akey);

            rprint!("bob shared generation| ");
            let bkey = shared_generation(alice_rem, bob_sec, &sp);
            rprintln!("key {}.", bkey);

            assert_eq!(akey, bkey);
            rprintln!("\nkey= {}", akey);

            let flags = key_typ(akey);

            rprint!("KeyType");
            for t in [Odd, Even, Prime, SafePrime] {
                let tu8 = t.clone() as u8;
                if flags & tu8 == tu8 {
                    rprint!("|");
                    rprint!("{t}",);
                }
            }
            rprintln!("\n");

            match flags {
                // as long as gen < prime, rem cannot be zero
                // otherwise check for instance 15^17 % 5
                0 => panic!(),
                1 => (1000, 0, 0, MAX_AMOUNT),             // odd, blue
                2 => (1000, 0, MAX_AMOUNT, 0),             // even, green
                5 => (1200, HALF_AMOUNT, HALF_AMOUNT, 0),  // odd prime, yellow
                6 => (1500, HALF_AMOUNT, 0, HALF_AMOUNT),  // even prime, purple
                13 => (2500, 0, HALF_AMOUNT, HALF_AMOUNT), // safe prime, always odd, cyan
                _ => panic!(),
            }
        } else {
            (150u32, MAX_AMOUNT, 0, 0)
        };

        set_duty(&pwm0, CHRED, r_amount);
        set_duty(&pwm0, CHBLUE, b_amount);
        set_duty(&pwm0, CHGREEN, g_amount);
        del.delay_ms(d_amount);
    }
}

#[derive(PartialEq)]
enum CheckCondition {
    GeneratorLessThenTwo,
    GeneratorGreaterOrEqualToPrime,
    Ok,
}

fn cond_ck(gen: u16, sp: u16) -> CheckCondition {
    return if gen < 2 {
        GeneratorLessThenTwo
    } else if gen < sp {
        Ok
    } else {
        GeneratorGreaterOrEqualToPrime
    };
}

fn private_generation(rng: &mut Rng, gen: &[u8], sp: &[u8]) -> (u8, u16) {
    let secret = rng.random_u8();

    let mut pow = pow(gen, secret);
    let rem = rem(pow.as_slice_mut(), sp);

    (secret, rem)
}

/// `rrem` — remote remainder, that side remainder
/// `losec` — local secret, this side secret
fn shared_generation(rrem: u16, losec: u8, sp: &[u8]) -> u16 {
    let rrem = to_decimals(rrem);
    let mut pow = pow(rrem.as_slice(), losec);

    rem(pow.as_slice_mut(), &sp)
}

fn set_duty(pwm: &Pwm<PWM0>, ch: Channel, amount: u16) {
    if amount == 0 {
        if pwm.duty_on(ch) != 0 {
            pwm.set_duty_on(ch, MAX_DUTY);
        }
    } else {
        pwm.set_duty_on(ch, amount);
    }
}

use core::ops::BitOr;

#[repr(u8)]
#[derive(Clone)]
enum KeyType {
    Zero = 0,
    Odd = 1,
    Even = 2,
    Prime = 4,
    SafePrime = 13, // 1101, always odd
    #[allow(dead_code)]
    SgPrime = 20, // 10100, can be even
    #[allow(dead_code)]
    SgAndSafePrime = 29, // 11101, always odd
}

impl BitOr for KeyType {
    type Output = KeyType;

    fn bitor(self, rhs: Self) -> KeyType {
        let add = self as u8 | rhs as u8;
        unsafe { core::mem::transmute::<u8, KeyType>(add) }
    }
}

use core::fmt::{Display, Formatter, Result};
impl Display for KeyType {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let text = match self {
            Zero => "Zero",
            Odd => "Odd",
            Even => "Even",
            Prime => "Prime",
            SafePrime => "SafePrime",
            Self::SgPrime => "SgPrime",
            Self::SgAndSafePrime => "SgAndSafePrime",
        };

        f.write_str(text)
    }
}

use crate::KeyType::{Even, Odd, Prime, SafePrime, Zero};
fn key_typ(num: u16) -> u8 {
    let typ = match prime_ck(num) {
        PrimeCk::Zero => Zero,
        PrimeCk::Two => Prime | Even,
        PrimeCk::Odd => Odd,
        PrimeCk::Even => Even,
        PrimeCk::Prime => match prime_ck((num - 1) >> 1) {
            PrimeCk::Prime => SafePrime,
            _ => Prime | Odd,
        },
    };

    typ as u8
}

#[allow(non_camel_case_types)]
type decimalsU16 = ([u8; MAX_U16_PLACES], usize);

#[allow(non_camel_case_types)]
type decimalsMax = (maxPlaces, usize);

#[allow(non_camel_case_types)]
type maxPlaces = [u8; MAX_PLACES];

// xₙ₊₁ = ½(xₙ+S÷xₙ)
fn herons_sqrt(num: u16) -> i32 {
    let num = num as i32;

    if num == 1 || num == 0 {
        return num;
    }

    let mut cur = num >> 1;

    loop {
        let nex = (cur + num / cur) >> 1;

        if nex >= cur {
            break;
        }

        cur = nex;
    }

    cur
}

#[derive(PartialEq)]
enum PrimeCk {
    Zero = 0,
    Odd = 1,
    Even = 2,
    Prime = 4,
    Two = 14,
}

// 1 < a ≤ b < num, num = a×b = √num×√num
//  ⇒ a=b=√num ∨ a < b ⇒ a < √num ∧ b > √num
fn prime_ck(num: u16) -> PrimeCk {
    match num {
        0 => return PrimeCk::Zero,
        1 => return PrimeCk::Odd,
        2 => return PrimeCk::Two,
        x if x & 1 != 1 => return PrimeCk::Even,
        _ => {}
    }

    let sqrt = herons_sqrt(num);

    let num = num as i32;
    for i in 2..=sqrt {
        if num % i == 0 {
            return PrimeCk::Odd;
        }
    }

    PrimeCk::Prime
}

trait AsSlice {
    fn as_slice(&self) -> &[u8];
}

trait AsSliceMut {
    fn as_slice_mut(&mut self) -> &mut [u8];
}

impl AsSlice for decimalsU16 {
    fn as_slice(&self) -> &[u8] {
        &self.0[..self.1]
    }
}

impl AsSliceMut for decimalsMax {
    fn as_slice_mut(&mut self) -> &mut [u8] {
        &mut self.0[..self.1]
    }
}

/// converts number to decimal places
fn to_decimals(num: u16) -> decimalsU16 {
    let mut decimals = [0; MAX_U16_PLACES];
    let mut ix = 0;

    let mut num = num as i32;

    loop {
        let d = num % 10;
        decimals[ix] = d as u8;
        num = num / 10;

        ix += 1;
        if num == 0 {
            break;
        }
    }

    (decimals, ix)
}

fn from_decimals(decimals: &[u8]) -> u16 {
    let mut num = 0;
    let len = decimals.len();

    for ix in 0..len {
        let place = decimals[ix];
        if place == 0 {
            continue;
        }

        num += place as i32 * 10i32.pow(ix as u32);
    }

    num as u16
}

fn rem(dividend: &mut [u8], divisor: &[u8]) -> u16 {
    let mut wdsor = [0; MAX_PLACES];

    let mut end_len = dividend.len();
    let sor_len = divisor.len();

    let sor_hg_ix = sor_len - 1;

    while end_len > sor_len {
        let mut wr_ix = end_len - 1;

        let mut l_ix = wr_ix;
        let mut r_ix = sor_hg_ix;

        loop {
            let end_num = dividend[l_ix];
            let sor_num = divisor[r_ix];

            if end_num < sor_num {
                wr_ix -= 1;
                break;
            } else if end_num > sor_num {
                break;
            }

            if r_ix == 0 {
                break;
            }

            l_ix -= 1;
            r_ix -= 1;
        }

        let wdsor_len = wr_ix + 1;
        let mut sor_ix = sor_hg_ix;

        loop {
            wdsor[wr_ix] = divisor[sor_ix];

            if sor_ix == 0 {
                break;
            }

            sor_ix -= 1;
            wr_ix -= 1;
        }

        end_len = rem_cross(dividend, &wdsor, end_len, wdsor_len);
    }

    if end_len == sor_len {
        end_len = rem_cross(dividend, divisor, end_len, sor_len);
    }

    from_decimals(&dividend[..end_len])
}

fn rem_cross(end: &mut [u8], sor: &[u8], end_len: usize, sor_len: usize) -> usize {
    let mut takeover;
    let mut ix;

    loop {
        takeover = 0;
        ix = 0;

        while ix < end_len {
            let sor_num = if ix < sor_len {
                sor[ix]
            } else if takeover == 0 {
                break;
            } else {
                0
            } as i32;

            let mut end_num = end[ix] as i32;
            let total = sor_num + takeover;

            takeover = if end_num < total {
                end_num += 10;
                1
            } else {
                0
            };

            end[ix] = (end_num - total) as u8;
            ix += 1;
        }

        if takeover == 1 {
            ix = 0;
            takeover = 0;

            let mut not_len = 0;

            while ix < sor_len && ix < end_len {
                let correction = end[ix] as i32 + sor[ix] as i32;

                let one = ones(correction, &mut takeover);
                end[ix] = one;

                if one == 0 {
                    not_len += 1;
                } else {
                    not_len = 0;
                }

                ix += 1;
            }

            return if not_len == ix { 1 } else { ix - not_len };
        }
    }
}

fn pow(base: &[u8], pow: u8) -> decimalsMax {
    let mut aux1 = [0; MAX_PLACES];

    if pow == 0 {
        aux1[0] = 1;
        return (aux1, 1);
    }

    let base_len = base.len();
    for ix in 0..base_len {
        aux1[ix] = base[ix]
    }

    if pow == 1 {
        return (aux1, base_len);
    }

    let mut aux2 = [0; MAX_PLACES];

    let mut steps = [0; 7];
    let mut wr_ix = 0;
    let mut step = pow;

    loop {
        steps[wr_ix] = step;
        step >>= 1;

        // `pow = 1` and `pow = 0` solved above
        if step == 1 {
            break;
        }

        wr_ix += 1;
    }

    let mut mcand = &mut aux1;
    let mut sum = &mut aux2;

    let mut mcand_len = base_len;
    let mut sum_len = 0;

    let mut ixes = (0..=wr_ix).rev();

    loop {
        let re_ix = unsafe { ixes.next().unwrap_unchecked() };

        for off in 0..mcand_len {
            sum_len = muladd(&mcand[0..mcand_len], mcand[off], sum, off);
        }

        if steps[re_ix] & 1 == 1 {
            clear_swap(&mut mcand, mcand_len, &mut sum);
            mcand_len = sum_len;

            for off in 0..base_len {
                sum_len = muladd(&mcand[0..mcand_len], base[off], sum, off);
            }
        }

        if re_ix == 0 {
            return (*sum, sum_len);
        }

        clear_swap(&mut mcand, mcand_len, &mut sum);
        mcand_len = sum_len;
    }
}

fn clear_swap<'a>(mcand: &mut &'a mut maxPlaces, mcand_len: usize, sum: *mut &'a mut maxPlaces) {
    for ix in 0..mcand_len {
        mcand[ix] = 0;
    }

    let swap: *mut maxPlaces = *mcand;

    unsafe {
        *mcand = *sum;
        *sum = swap.as_mut().unwrap();
    }
}

fn muladd(mcand: &[u8], mpler: u8, sum: &mut [u8], base_off: usize) -> usize {
    let mut sum_max_ix = 0;

    let mut ix = 0;
    let mcand_len = mcand.len();

    loop {
        let prod = mpler as i32 * mcand[ix] as i32;

        let max_wr_ix = sumadd(prod, sum, base_off + ix);

        if max_wr_ix > sum_max_ix {
            sum_max_ix = max_wr_ix
        };

        ix += 1;

        if ix == mcand_len {
            break;
        }
    }

    sum_max_ix + 1
}

fn sumadd(mut addend: i32, sum: &mut [u8], mut off: usize) -> usize {
    let mut takeover = 0;

    loop {
        let augend = sum[off];

        sum[off] = ones(augend as i32 + addend, &mut takeover);

        if takeover == 0 {
            break;
        } else {
            addend = 0;
            off += 1;
        }
    }

    off
}

fn ones(num: i32, takeover_ref: &mut i32) -> u8 {
    let mut takeover_val = *takeover_ref;
    let total = num + takeover_val;

    takeover_val = total / 10;
    *takeover_ref = takeover_val;

    (total - takeover_val * 10) as u8
}

//  cargo embed --target thumbv7em-none-eabihf
//  cargo build --target thumbv7em-none-eabihf
//  cargo build --target thumbv7em-none-eabihf --release

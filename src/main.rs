use std::io::{BufReader, Cursor};
use std::io::Read;

use byteorder::{ByteOrder, LittleEndian};
use reqwest::header::ACCEPT_ENCODING;
use ruzstd::StreamingDecoder;
use tokio::io::AsyncReadExt;

#[derive(Debug)]
enum SignalType {
    AdsbIcao,
    AdsbIcaoNt,
    AdsrIcao,
    TisbIcao,
    Adsc,
    Mlat,
    Other,
    ModeS,
    AdsbOther,
    AdsrOther,
    TisbTrackfile,
    TisbOther,
    ModeAc,
    Unknown,
}

impl SignalType {
    fn is_adsb(&self) -> bool {
        match self {
            SignalType::AdsbIcao => true,
            SignalType::AdsbIcaoNt => true,
            SignalType::AdsbOther => true,
            _ => false,
        }
    }

    fn is_adsr(&self) -> bool {
        match self {
            SignalType::AdsrIcao => true,
            SignalType::AdsrOther => true,
            _ => false,
        }
    }

    fn is_tisb(&self) -> bool {
        match self {
            SignalType::TisbIcao => true,
            SignalType::TisbTrackfile => true,
            SignalType::TisbOther => true,
            _ => false,
        }
    }
}

#[derive(Default, Debug)]
struct Aircraft {
    hex: String,
    seen_pos: Option<f32>,
    seen: Option<f32>,
    lon: Option<f32>,
    lat: Option<f32>,
    baro_rate: Option<i32>,
    geom_rate: Option<i32>,
    alt_baro: Option<i32>,
    alt_baro_label: Option<&'static str>,
    alt_geom: Option<i32>,
    nav_altitude_mcp: Option<u32>,
    nav_altitude_fms: Option<u32>,
    nav_qnh: Option<f32>,
    nav_heading: Option<f32>,
    squawk: Option<String>,
    gs: Option<f32>,
    mach: Option<f32>,
    roll: Option<f32>,
    track: Option<f32>,
    track_rate: Option<f32>,
    mag_heading: Option<f32>,
    true_heading: Option<f32>,
    wd: Option<i16>,
    ws: Option<i16>,
    oat: Option<i16>,
    tat: Option<i16>,
    tas: Option<u16>,
    ias: Option<u16>,
    rc: u16,
    messages: u16,
    message_rate: u16,
    category: Option<String>,
    nic: u8,
    nav_modes: Vec<&'static str>,
    emergency: Option<u8>,
    signal_type: Option<SignalType>,
    airground: u8,
    nav_altitude_src: Option<u8>,
    sil_type: u8,
    adsb_version: u8,
    adsr_version: u8,
    tisb_version: u8,
    nac_p: Option<u8>,
    nac_v: Option<u8>,
    sil: Option<u8>,
    gva: Option<u8>,
    sda: Option<u8>,
    nic_a: Option<u8>,
    nic_c: Option<u8>,
    flight: Option<String>,
    db_flags: u16,
    tail: String,
    registration: String,
    receiver_count: u8,
    rssi: f64,
    extra_flags: u8,
    nogps: u8,
    nic_baro: Option<u8>,
    alert1: Option<u8>,
    spi: Option<u8>,
    r_id: Option<String>,
}

#[derive(Debug)]
struct BinCraft {
    now: f64,
    stride: u32,
    global_ac_count_withpos: u32,
    globe_index: u32,
    south: i16,
    west: i16,
    north: i16,
    east: i16,
    messages: u32,
    receiver_lat: f64,
    receiver_lon: f64,
    aircraft: Vec<Aircraft>,
}

#[inline]
fn build_aircraft(
    data: &[u8],
    stride: u32,
    use_message_rate: bool,
) -> Aircraft {
    let mut aircraft = Aircraft::default();
    let mut data = data.to_vec();

    let stride = stride as usize;

    let mut u32 = Vec::<u32>::with_capacity(stride / 4);
    let mut s32 = Vec::<i32>::with_capacity(stride / 4);
    let mut u16 = Vec::<u16>::with_capacity(stride / 2);
    let mut s16 = Vec::<i16>::with_capacity(stride / 2);

    for i in 0..stride / 4 {
        u32.push(LittleEndian::read_u32(&data[i * 4..i * 4 + 4]));
        s32.push(LittleEndian::read_i32(&data[i * 4..i * 4 + 4]));
    }

    for i in 0..stride / 2 {
        u16.push(LittleEndian::read_u16(&data[i * 2..i * 2 + 2]));
        s16.push(LittleEndian::read_i16(&data[i * 2..i * 2 + 2]));
    }

    let t = s32[0] & 1 << 24;

    aircraft.hex = format!("{:06x}", 16777215 & s32[0]);
    aircraft.hex = if t != 0 { "~".to_string() + &aircraft.hex } else { aircraft.hex };

    aircraft.seen_pos = Some(u16[2] as f32 / 10.0);
    aircraft.seen = Some(u16[3] as f32 / 10.0);
    aircraft.lon = Some(s32[2] as f32 / 1e6);
    aircraft.lat = Some(s32[3] as f32 / 1e6);
    aircraft.baro_rate = Some(8 * s16[8] as i32);
    aircraft.geom_rate = Some(8 * s16[9] as i32);
    aircraft.alt_baro = Some(25 * s16[10] as i32);
    aircraft.alt_geom = Some(25 * s16[11] as i32);
    aircraft.nav_altitude_mcp = Some((4.0 * u16[12] as f32) as u32);
    aircraft.nav_altitude_fms = Some((4.0 * u16[13] as f32) as u32);
    aircraft.nav_qnh = Some(s16[14] as f32 / 10.0);
    aircraft.nav_heading = Some(s16[15] as f32 / 90.0);

    let s = format!("{:04x}", u16[16]);

    aircraft.squawk =
        if s.chars().nth(0).unwrap().to_digit(10).unwrap() > 9 {
            Some(
                format!(
                    "{}{}{}{}",
                    s.chars().nth(0).unwrap().to_digit(16).unwrap(),
                    &s[1..2],
                    &s[2..3],
                    &s[3..4],
                ),
            )
        } else {
            Some(s)
        };

    aircraft.gs = Some(s16[17] as f32 / 10.0);
    aircraft.mach = Some(s16[18] as f32 / 1e3);
    aircraft.roll = Some(s16[19] as f32 / 100.0);
    aircraft.track = Some(s16[20] as f32 / 90.0);
    aircraft.track_rate = Some(s16[21] as f32 / 100.0);
    aircraft.mag_heading = Some(s16[22] as f32 / 90.0);
    aircraft.true_heading = Some(s16[23] as f32 / 90.0);
    aircraft.wd = Some(s16[24]);
    aircraft.ws = Some(s16[25]);
    aircraft.oat = Some(s16[26]);
    aircraft.tat = Some(s16[27]);
    aircraft.tas = Some(u16[28]);
    aircraft.ias = Some(u16[29]);
    aircraft.rc = u16[30];

    if use_message_rate {
        aircraft.message_rate = u16[31] / 10;
    } else {
        aircraft.messages = u16[31];
    }

    aircraft.category =
        if data[64] != 0 {
            Some(format!("{:02X}", data[64]))
        } else {
            None
        };

    aircraft.nic = data[65];

    let nav_modes = data[66];

    aircraft.nav_modes = Vec::new();
    aircraft.emergency = Some(15 & data[67]);

    let signal_type = (240 & data[67]) >> 4;

    aircraft.airground = 15 & data[68];
    aircraft.nav_altitude_src = Some((240 & data[68]) >> 4);
    aircraft.sil_type = 15 & data[69];
    aircraft.adsb_version = (240 & data[69]) >> 4;
    aircraft.adsr_version = 15 & data[70];
    aircraft.tisb_version = (240 & data[70]) >> 4;
    aircraft.nac_p = Some(15 & data[71]);
    aircraft.nac_v = Some((240 & data[71]) >> 4);
    aircraft.sil = Some(3 & data[72]);
    aircraft.gva = Some((12 & data[72]) >> 2);
    aircraft.sda = Some((48 & data[72]) >> 4);
    aircraft.nic_a = Some((64 & data[72]) >> 6);
    aircraft.nic_c = Some((128 & data[72]) >> 7);

    aircraft.flight =
        Some(
            String::from_utf8_lossy(&data[78..86])
                .trim_end_matches(char::from(0))
                .to_string(),
        );

    aircraft.db_flags = u16[43];

    aircraft.tail =
        String::from_utf8_lossy(&data[88..92])
            .trim_end_matches(char::from(0))
            .to_string();

    aircraft.registration =
        String::from_utf8_lossy(&data[92..104])
            .trim_end_matches(char::from(0))
            .to_string();

    aircraft.receiver_count = data[104];
    aircraft.rssi = 10.0 * ((data[105] as f64 * data[105] as f64 / 65025.0 + 1125e-8) as f64).log10();
    aircraft.extra_flags = data[106];

    aircraft.nogps = 1 & aircraft.extra_flags;

    if aircraft.nogps != 0 && s32[3] == 2147483647 {
        data[73] |= 64;
        data[73] |= 16;
    }

    aircraft.nic_baro = Some(1 & data[73]);
    aircraft.alert1 = Some(2 & data[73]);
    aircraft.spi = Some(4 & data[73]);

    if 8 & data[73] == 0 { aircraft.flight = None; }
    if 16 & data[73] == 0 { aircraft.alt_baro = None; }
    if 32 & data[73] == 0 { aircraft.alt_geom = None; }
    if 64 & data[73] == 0 {
        aircraft.lat = None;
        aircraft.lon = None;
        aircraft.seen_pos = None;
    }
    if 128 & data[73] == 0 { aircraft.gs = None; }

    if 1 & data[74] == 0 { aircraft.ias = None; }
    if 2 & data[74] == 0 { aircraft.tas = None; }
    if 4 & data[74] == 0 { aircraft.mach = None; }
    if 8 & data[74] == 0 {
        aircraft.track = None;
        //aircraft.calc_track = None;
    }
    if 16 & data[74] == 0 { aircraft.track_rate = None; }
    if 32 & data[74] == 0 { aircraft.roll = None; }
    if 64 & data[74] == 0 { aircraft.mag_heading = None; }
    if 128 & data[74] == 0 { aircraft.true_heading = None; }

    if 1 & data[75] == 0 { aircraft.baro_rate = None; }
    if 2 & data[75] == 0 { aircraft.geom_rate = None; }

    if 4 & data[75] == 0 { aircraft.nic_a = None; }
    if 8 & data[75] == 0 { aircraft.nic_c = None; }
    if 16 & data[75] == 0 { aircraft.nic_baro = None; }
    if 32 & data[75] == 0 { aircraft.nac_p = None; }
    if 64 & data[75] == 0 { aircraft.nac_v = None; }
    if 128 & data[75] == 0 { aircraft.sil = None; }

    if 1 & data[76] == 0 { aircraft.gva = None; }
    if 2 & data[76] == 0 { aircraft.sda = None; }
    if 4 & data[76] == 0 { aircraft.squawk = None; }
    if 8 & data[76] == 0 { aircraft.emergency = None; }
    if 16 & data[76] == 0 { aircraft.spi = None; }
    if 32 & data[76] == 0 { aircraft.nav_qnh = None; }
    if 64 & data[76] == 0 { aircraft.nav_altitude_mcp = None; }
    if 128 & data[76] == 0 { aircraft.nav_altitude_fms = None; }

    if 1 & data[77] == 0 { aircraft.nav_altitude_src = None; }
    if 2 & data[77] == 0 { aircraft.nav_heading = None; }
    if 4 & data[77] == 0 { aircraft.nav_modes = Vec::new(); }
    if 8 & data[77] == 0 { aircraft.alert1 = None; }
    if 16 & data[77] == 0 {
        aircraft.ws = None;
        aircraft.wd = None;
    }
    if 32 & data[77] == 0 {
        aircraft.oat = None;
        aircraft.tat = None;
    }

    if aircraft.airground == 1 {
        aircraft.alt_baro_label = Some("ground");
    }

    if 4 & data[77] != 0 {
        aircraft.nav_modes = vec![];

        if 1 & nav_modes != 0 { aircraft.nav_modes.push("autopilot"); }
        if 2 & nav_modes != 0 { aircraft.nav_modes.push("vnav"); }
        if 4 & nav_modes != 0 { aircraft.nav_modes.push("alt_hold"); }
        if 8 & nav_modes != 0 { aircraft.nav_modes.push("approach"); }
        if 16 & nav_modes != 0 { aircraft.nav_modes.push("lnav"); }
        if 32 & nav_modes != 0 { aircraft.nav_modes.push("tcas"); }
    }

    aircraft.signal_type =
        Some(
            match signal_type {
                0 => SignalType::AdsbIcao,
                1 => SignalType::AdsbIcaoNt,
                2 => SignalType::AdsrIcao,
                3 => SignalType::TisbIcao,
                4 => SignalType::Adsc,
                5 => SignalType::Mlat,
                6 => SignalType::Other,
                7 => SignalType::ModeS,
                8 => SignalType::AdsbOther,
                9 => SignalType::AdsrOther,
                10 => SignalType::TisbTrackfile,
                11 => SignalType::TisbOther,
                12 => SignalType::ModeAc,
                _ => SignalType::Unknown,
            },
        );

    aircraft
}

fn parse_adsb(data: Vec<u8>) {
    let u32 = &data[0..44];
    let now = LittleEndian::read_u32(&u32[0..4]) as f64 / 1e3 + 4294967.296 * (LittleEndian::read_u32(&u32[4..8]) as f64);
    let stride = LittleEndian::read_u32(&u32[8..12]);
    let global_ac_count_withpos = LittleEndian::read_u32(&u32[12..16]);
    let globe_index = LittleEndian::read_u32(&u32[16..20]);

    let limits = &data[20..28];
    let south = LittleEndian::read_i16(&limits[0..2]);
    let west = LittleEndian::read_i16(&limits[2..4]);
    let north = LittleEndian::read_i16(&limits[4..6]);
    let east = LittleEndian::read_i16(&limits[6..8]);

    let messages = LittleEndian::read_u32(&u32[28..32]);

    let s32 = &data[32..32 + stride as usize];
    let receiver_lat = LittleEndian::read_i32(&s32[32..36]) as f64 / 1e6;
    let receiver_lon = LittleEndian::read_i32(&s32[36..40]) as f64 / 1e6;

    let bin_craft_version = LittleEndian::read_u32(&u32[40..44]);

    let mut aircraft = Vec::new();

    for off in (stride as usize..data.len()).step_by(stride as usize) {
        aircraft.push(
            build_aircraft(
                &data[off..off + stride as usize],
                stride,
                globe_index != 0 && bin_craft_version >= 20220916,
            ),
        );
    }

    let data = BinCraft {
        now,
        stride,
        global_ac_count_withpos,
        globe_index,
        south,
        west,
        north,
        east,
        messages,
        receiver_lat,
        receiver_lon,
        aircraft,
    };

    println!("{:?}", data.aircraft.len());
}

fn main() {
    parse_adsb(
        zstd::decode_all(
            BufReader::new(&include_bytes!("../dump-mil.bin")[..]),
        ).unwrap(),
    );
}

//#[tokio::main]
//async fn main() -> Result<(), Box<dyn std::error::Error>> {
//    let url = "https://globe.adsbexchange.com/re-api/?binCraft&zstd&box=44.006377,46.049806,7.711479,15.364432";
//    let client = reqwest::Client::new();
//
//    let mut res = client
//        .get(url)
//        .send()
//        .await?;
//
//    let data = res.bytes().await?;
//    let data = data.to_vec();
//    let mut data = BufReader::new(data.as_slice());
//
//    let mut decoder =
//        StreamingDecoder::new(&mut data).unwrap();
//
//    let mut result = Vec::new();
//    decoder.read_to_end(&mut result).unwrap();
//
//    println!("{:?}", result);
//
//    Ok(())
//}

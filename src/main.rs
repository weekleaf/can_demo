use socketcan::{CanSocket, CanFrame, CanDataFrame, Socket, EmbeddedFrame, StandardId, Frame};
use std::time::Duration;
use tokio::sync::Mutex;
use futures_util::{SinkExt, StreamExt};
use lazy_static::lazy_static;
use pid::Pid;

// fn main() {
//     let socket = CanSocket::open("can0").expect("Failed to open CAN");

//     let p_des: u16 = 0;
//     let v_des: u16 = 0;
//     let kp: u16 = 0;
//     let kd: u16 = 0;
//     let t_ff: u16 = 10; 

//     let frame = CanDataFrame::new(
//         StandardId::new(0x10).expect("CAN id is FKING"),
//         &[
//             (p_des >> 8) as u8,
//             (p_des & 0xFF) as u8,
//             (v_des & 0x0FF0) as u8, 
//             ((v_des & 0x000F) | (kp & 0x0F00)) as u8,
//             (kp & 0xFF) as u8,
//             (kd & 0x0FF0) as u8,
//             ((kd & 0x000F) | (t_ff & 0x0F00)) as u8,
//             (t_ff & 0xFF) as u8
//         ],
//     )
//     .expect("Failed to create CAN frame");

//     let frame = CanFrame::Data(frame);

//     loop {
//         socket.write_frame(&frame).expect("Failed to send CAN frame");
//     }
    
// }

lazy_static! { static ref MOTOR1: Mutex<Motor> = Mutex::new(Motor::new()); }

fn limit_min_max(x: &mut f32, min: f32, max: f32) {
    if *x <= min {
        *x = min;
    } else if *x > max {
        *x = max;
    }
}

fn float_2_uint(mut x: f32, x_min: f32, x_max: f32, bits: u8) -> u16 {
    limit_min_max(&mut x, x_min, x_max);
    let span = x_max - x_min;
    let data_norm = (x - x_min) / span;
    (data_norm * (((1 << bits) - 1) as f32)) as u16
}

fn uint_2_float(x: u16, x_min: f32, x_max: f32, bits: u8) -> f32 {
    let span =  x_max - x_min;
    let data_norm = x as f32 / (((1 << bits) - 1) as f32);
    data_norm * span + x_min
}

struct Motor {
    read_pos: f32,
    read_vel: f32,
    read_t: f32,
}

impl Motor {
    fn new() -> Self {
        Self {
            read_pos: 0f32,
            read_vel: 0f32,
            read_t: 0f32,
        }
    }

    fn handle_frame(&mut self, frame: &CanDataFrame) {
        let mut read_data  = frame.data();
        let read_pos_uint = ((read_data[1]) as u16) << 8 | (read_data[2] as u16);
        let read_vel_uint = ((read_data[3] as u16)) << 4 | ((read_data[4] as u16)) >> 4;
        let read_t_uint = ((read_data[4] & 0xF) as u16) << 8 | (read_data[5] as u16);

        self.read_pos = uint_2_float(read_pos_uint, -3.141593, 3.141593, 16);
        self.read_vel = uint_2_float(read_vel_uint, -50f32, 50f32, 12);
        self.read_t = uint_2_float(read_t_uint, -10f32, 10f32, 12);

        // println!("pos_uint : {}, vel_uint : {}, t_uint : {}", read_pos_uint, read_vel_uint, read_t_uint);
        // println!("pos : {}, vel : {}, t : {}", self.read_pos, self.read_vel, self.read_t);

        
    }
}

async fn rotate_motor<T>(motor: &Motor, tx: &mut T/*, t_ff: f32*/)
where
    T: futures_util::sink::Sink<CanFrame, Error = socketcan::Error> + Unpin,
{
    // let t_ff_uint = float_2_uint(t_ff, -10f32, 10f32, 12);

    let target: f32 = 3.0;
    let mut pos_error = target - motor.read_pos;

    if pos_error > 3.141593 {
        pos_error -= 2.0 * 3.141593;
    } else if  pos_error < -3.141593 {
        pos_error += 2.0 * 3.141593;
    }

    let mut pid: Pid<f32> =  Pid::new(0.0, 5.0);
    pid.p(0.1, 100.0);
    pid.i(0.0, 100.0);
    pid.d(0.0, 100.0);
    let output = pid.next_control_output(pos_error);
    
    let final_output = float_2_uint(output.output, -10.0, 10.0, 12);

    println!("===========================================================================");
    println!("output : {}, read_pos : {}", output.output, motor.read_pos);
    println!("final_output : {}", final_output);

    // let p_des: f32 = 0.0;
    // let v_des: f32 = 0.0;
    // let kp: f32 = 0.0;
    // let kd: f32 = 0.0;
    // let t_ff: f32 = 10.0;

    // let p_des_uint = float_2_uint(p_des, -3.141593, 3.141593, 16);
    // let v_des_uint = float_2_uint(v_des, -50.0, 50.0, 12);
    // let kp_uint = float_2_uint(kp, 0.0, 500.0, 12);
    // let kd_uint = float_2_uint(kd, 0.0, 5.0, 12);
    // let t_ff_uint = float_2_uint(t_ff, -10.0, 10.0, 12);

    let id: u16 = 0x10;
    let frame = CanDataFrame::new(
        StandardId::new(id).unwrap(),
        &[
            // a problem! do not to use!!!
            // (p_des_uint >> 8) as u8,
            // (p_des_uint & 0xFF) as u8,
            // (v_des_uint & 0x0FF0) as u8, 
            // ((v_des_uint & 0x000F) | (kp_uint & 0x0F00)) as u8,
            // (kp_uint & 0xFF) as u8,
            // (kd_uint & 0x0FF0) as u8,
            // ((kd_uint & 0x000F) | (t_ff_uint & 0x0F00)) as u8,
            // (t_ff_uint & 0xFF) as u8
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            (final_output >> 8) as u8,
            (final_output & 0xFF) as u8
        ],
    )
    .unwrap();
    tx.send(socketcan::CanFrame::Data(frame)).await.unwrap();
}

fn generate_reboot_command() -> CanFrame {
    let id: u16 = 0x10;
    socketcan::CanFrame::Data(
        CanDataFrame::new(
            StandardId::new(id).unwrap(),
            &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
        )
        .unwrap(),
    )
}

#[tokio::main]
async fn main() {
    let can_name = "can0".to_string();
    let mut bus = socketcan::tokio::CanSocket::open(&can_name).unwrap();
    let (mut tx, mut rx) = bus.split();
    tokio::time::sleep(Duration::from_millis(100)).await;
    tx.send(generate_reboot_command()).await.unwrap();
    tokio::time::sleep(Duration::from_millis(100)).await;

    tokio::spawn(async move {
        let mut rx = rx;
        loop {
            if let Some(Ok(socketcan::CanFrame::Data(frame))) = rx.next().await {
                if frame.raw_id() == 0x01 {
                    let mut motor1 = MOTOR1.lock().await;
                    motor1.handle_frame(&frame);
                }
            }
        }
    });
    loop {
        tokio::time::sleep(Duration::from_millis(5)).await;
        let mut motor1 = MOTOR1.lock().await;
        rotate_motor(&motor1, &mut tx).await;
    }
}

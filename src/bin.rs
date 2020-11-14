// example of usage
pub fn main() {
    use intrepid::ControllerBuilder;
    let mut pid = ControllerBuilder::new_with_target(1000.0)
        .with_p_gain(0.01)
        .with_i_gain(0.01)
        .with_d_gain(0.01)
        .build()
        .expect("well formed parameters");
    let mut prev = 1.0;
    for i in 0..=100 {
        let output = pid.compute(prev, 10.0);
        prev = output;
        println!("{} : {}", i, output)
    }
}

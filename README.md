# intrepid

[![Build Status](https://travis-ci.org/agourlay/intrepid.svg?branch=master)](https://travis-ci.org/agourlay/intrepid)

This library implements a simple [PID controller](https://en.wikipedia.org/wiki/PID_controller) in Rust.

## features

- derivative on error by default
- derivative on measurement (optional)
- max output value
- reset


## usage

The library does not schedule measurements nor track the time elapsed in between measurements.

Therefore, it requires the time difference since the previous measurement as an input to `compute`

```rust
use intrepid::ControllerBuilder;

let mut pid = ControllerBuilder::new_with_target(1000.0)
    .with_p_gain(0.01)
    .with_i_gain(0.01)
    .with_d_gain(0.01)
    .build()
    .expect("well formed parameters");

let mut prev = 1.0;

for i in 0..=100 {
    let output = pid.compute(prev, 10.0); // the second argument is the time elapsed between measurements
    prev = output;
    println!("{} : {}", i, output)
}
```

## learning resources

- http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
- https://www.youtube.com/watch?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y&v=wkfEZmsQqiA

## todos

- handle saturation with an integrator anti-windup
- implement a real world example (cruise control like)

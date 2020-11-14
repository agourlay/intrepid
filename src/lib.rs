// inspired by http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

pub struct ControllerBuilder {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub target: f64,
    pub max_output_value: f64,
    pub derivative_on_measurement: bool,
}

impl ControllerBuilder {
    pub fn new_with_target(target: f64) -> Self {
        ControllerBuilder {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            target,
            max_output_value: std::f64::MAX,
            derivative_on_measurement: false,
        }
    }

    pub fn with_p_gain(mut self, kp: f64) -> Self {
        self.kp = kp;
        self
    }

    pub fn with_d_gain(mut self, kd: f64) -> Self {
        self.kd = kd;
        self
    }

    pub fn with_i_gain(mut self, ki: f64) -> Self {
        self.ki = ki;
        self
    }

    pub fn with_derivative_on_measurement(mut self) -> Self {
        self.derivative_on_measurement = true;
        self
    }

    pub fn with_max_output_value(mut self, max_output_value: f64) -> Self {
        self.max_output_value = max_output_value;
        self
    }

    pub fn build(&self) -> Result<Controller, String> {
        if self.kp == 0.0 && self.ki == 0.0 && self.kd == 0.0 {
            Err("All gains cannot be zero at the same time".to_string())
        } else {
            Ok(Controller::new(
                self.kp,
                self.ki,
                self.kd,
                self.target,
                self.derivative_on_measurement,
                self.max_output_value,
            ))
        }
    }
}

pub struct Controller {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub target: f64,
    pub max_output_value: f64,
    pub derivative_on_measurement: bool,
    pub sum_error: f64,
    pub last_error: f64,
    pub last_input: f64,
}

impl Controller {
    fn new(
        kp: f64,
        ki: f64,
        kd: f64,
        target: f64,
        derivative_on_measurement: bool,
        max_output_value: f64,
    ) -> Controller {
        Controller {
            kp,
            ki,
            kd,
            target,
            derivative_on_measurement,
            max_output_value,
            sum_error: 0.0,
            last_error: 0.0,
            last_input: 0.0,
        }
    }

    pub fn set_target(mut self, target: f64) -> Self {
        self.target = target;
        self
    }

    pub fn reset(mut self) -> Self {
        self.last_error = 0.0;
        self.last_input = 0.0;
        self.sum_error = 0.0;
        self
    }

    pub fn compute(&mut self, input: f64, time_elapsed: f64) -> f64 {
        let current_error = self.target - input;
        let current_sum_error = self.sum_error + current_error * time_elapsed as f64;
        let derivative_error = if time_elapsed == 0.0 {
            0.0 // first round
        } else if self.derivative_on_measurement {
            (input - self.last_input) / time_elapsed as f64
        } else {
            (current_error - self.last_error) / time_elapsed as f64
        };

        // update internal state
        self.last_error = current_error;
        self.last_input = input;
        self.sum_error = current_sum_error;

        let p_term = self.kp * current_error;
        let i_term = self.ki * current_sum_error;
        let d_term = self.kd * derivative_error;

        let output = p_term + i_term + d_term;
        if output > self.max_output_value {
            self.max_output_value
        } else {
            output
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::ControllerBuilder;

    #[test]
    fn only_proportional() {
        let mut controller = ControllerBuilder::new_with_target(10.0)
            .with_p_gain(1.0)
            .build()
            .unwrap();

        // output is constant with measurement
        let output_1 = controller.compute(2.0, 100.0);
        assert_eq!(8.0, output_1);
        let output_2 = controller.compute(2.0, 100.0);
        assert_eq!(8.0, output_2);
    }

    #[test]
    fn only_integral() {
        let mut controller = ControllerBuilder::new_with_target(10.0)
            .with_i_gain(0.5)
            .build()
            .unwrap();

        // output changes over time for the same measurement as the sum of error grows
        let output_1 = controller.compute(2.0, 1.0);
        assert_eq!(4.0, output_1);
        let output_2 = controller.compute(2.0, 1.0);
        assert_eq!(8.0, output_2);
    }

    #[test]
    fn only_derivative_on_error() {
        let mut controller = ControllerBuilder::new_with_target(10.0)
            .with_d_gain(0.5)
            .build()
            .unwrap();

        // output grows with the error's derivative
        let output_1 = controller.compute(9.0, 1.0);
        assert_eq!(0.5, output_1);
        let output_2 = controller.compute(7.0, 1.0);
        assert_eq!(1.0, output_2);
        let output_3 = controller.compute(4.0, 1.0);
        assert_eq!(1.5, output_3);
    }

    #[test]
    fn only_derivative_on_measurement() {
        let mut controller = ControllerBuilder::new_with_target(10.0)
            .with_d_gain(0.5)
            .with_derivative_on_measurement()
            .build()
            .unwrap();

        // output grows with the input's derivative
        let output_1 = controller.compute(1.0, 1.0);
        assert_eq!(0.5, output_1);
        let output_2 = controller.compute(3.0, 1.0);
        assert_eq!(1.0, output_2);
        let output_3 = controller.compute(6.0, 1.0);
        assert_eq!(1.5, output_3);
    }

    #[test]
    fn with_max_output_value() {
        let mut controller = ControllerBuilder::new_with_target(1000.0)
            .with_max_output_value(2.0)
            .with_p_gain(10.0)
            .build()
            .unwrap();

        // output is constant with measurement
        let output_1 = controller.compute(1.0, 100.0);
        assert_eq!(2.0, output_1);
        let output_2 = controller.compute(1.0, 100.0);
        assert_eq!(2.0, output_2);
    }
}

//package org.firstinspires.ftc.teamcode.library.drivetrain.swerveDrive;
//
//public class PID {
//    private double p = 0, i = 0, d = 0;
//    public PID(double p, double i, double d) {
//        this.p = p;
//        this.i = i;
//        this.d = d;
//    }
//
//    public double update() {
//        double Kp = someValue;
//        Ki = someValue;
//        Kd = someValue;
//
//        reference = someValue;
//
//        integralSum = 0;
//
//        lastError = 0;
//
//// Elapsed timer class from SDK, please use it, it's epic
//        ElapsedTime timer = new ElapsedTime();
//
//        while (setPointIsNotReached) {
//
//
//            // obtain the encoder position
//            encoderPosition = armMotor.getPosition();
//            // calculate the error
//            error = reference - encoderPosition;
//
//            // rate of change of the error
//            derivative = (error - lastError) / timer.seconds();
//
//            // sum of all error over time
//            integralSum = integralSum + (error * timer.seconds());
//
//            out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
//
//            armMotor.setPower(out);
//
//            lastError = error;
//
//            // reset the timer for next time
//            timer.reset();
//
//        }
//    }
//}

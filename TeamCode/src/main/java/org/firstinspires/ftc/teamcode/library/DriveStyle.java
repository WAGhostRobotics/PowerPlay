
package org.firstinspires.ftc.teamcode.library;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;

public class DriveStyle {

    private static double TRIGGER_MINIMUM = 0.1;

    public static void Tank(ArrayList<DcMotor> motors, double multiplier, double left, double right) {
        motors.get(0).setPower(multiplier * left);
        motors.get(1).setPower(multiplier * left);
        motors.get(2).setPower(multiplier * right);
        motors.get(3).setPower(multiplier * right);
    }

    /**
     * Move motors based on left/right joystick from gamepad.
     *
     * @param motors array of motors [lf, lb, rf, rb]
     */
    public static void Tank(ArrayList<DcMotor> motors, Gamepad gamepad) {
        Tank(motors, gear(gamepad), -gamepad.left_stick_y, -gamepad.right_stick_y);
    }

    /**
     * Move motors based on drone-style input
     *
     * @param motors array of motors [lf, lb, rf, rb]
     */
    public static void MecanumArcade(ArrayList<DcMotor> motors, double multiplier, double x, double y, double turn) {
        double[] motorPowerArray = MecanumTrigMath.inputsToMotors(x, y, turn);

        //switch x and turn
//        double[] motorPowerArray = MecanumTrigMath.inputsToMotors(turn, y, x);
        motors.get(0).setPower(multiplier * motorPowerArray[0]);
        motors.get(1).setPower(multiplier * motorPowerArray[1]);
        motors.get(2).setPower(multiplier * motorPowerArray[2]);
        motors.get(3).setPower(multiplier * motorPowerArray[3]);
    }

    /*
     * Move motors based on drone-style input from gamepad (left stick strafe, right stick turn)
     *
     * @param motors  array of motors [lf, lb, rf, rb]
     * @param gamepad the gamepad to control the robot with (gamepad1/gamepad2)
     */
    public static void MecanumArcade(ArrayList<DcMotor> motors, Gamepad gamepad, double slow) {
        //turning and x was working fine
        //y was reversed so made y coordinates positive
        MecanumArcade(motors, gear(gamepad),  gamepad.left_stick_x*slow, -gamepad.left_stick_y*slow, gamepad.right_stick_x*slow);
    }

    /**
     * Leo says: inwards left, outwards right (1/12/2019)
     * @param motors
     * @param multiplier
     * @param left
     * @param right
     * @param strafeLeft
     * @param strafeRight
     */
    public static void MecanumTank(ArrayList<DcMotor> motors, double multiplier, double left, double right, double strafeLeft, double strafeRight) {
        motors.get(0).setPower((multiplier * left) + (multiplier * strafeRight) - (multiplier * strafeLeft));
        motors.get(1).setPower((multiplier * left) - (multiplier * strafeRight) + (multiplier * strafeLeft));
        motors.get(2).setPower((multiplier * right) - (multiplier * strafeRight) + (multiplier * strafeLeft));
        motors.get(3).setPower((multiplier * right) + (multiplier * strafeRight) - (multiplier * strafeLeft));
    }

    /**
     * Move motors based on left/right joystick from gamepad.
     *
     * @param motors array of motors [lf, lb, rf, rb]
     */
    public static void MecanumTank(ArrayList<DcMotor> motors, Gamepad gamepad) {
        MecanumTank(motors, gear(gamepad), -gamepad.left_stick_y, -gamepad.right_stick_y, gamepad.left_trigger, gamepad.right_trigger);
    }

    /**
     * Shortcut for stopping all motors.
     *
     * @param motors array of motors [lf, lb, rf, rb]
     */
    public static void stop(ArrayList<DcMotor> motors) {
        motors.get(0).setPower(0);
        motors.get(1).setPower(0);
        motors.get(2).setPower(0);
        motors.get(3).setPower(0);
    }

    public static void driveWithType(ArrayList<DcMotor> motors, Gamepad gamepad, DriveType type, double slow) {
        switch (type) {
            case TANK:
                Tank(motors, gamepad);
                break;
            case MECANUMARCADE:
                MecanumArcade(motors, gamepad, slow);
                break;
            case MECANUMTANK:
                MecanumTank(motors, gamepad);
                break;
        }
    }

    private static double gear(Gamepad gamepad) {
        double gear;

        if (gamepad.left_trigger > TRIGGER_MINIMUM && gamepad.right_trigger > TRIGGER_MINIMUM) {
            gear = .25;
        } else if (gamepad.left_trigger > TRIGGER_MINIMUM || gamepad.right_trigger > TRIGGER_MINIMUM) {
            gear = .5;
        } else {
            gear = 1;
        }

        return gear;
    }

    public enum DriveType {
        TANK,
        MECANUMARCADE,
        MECANUMTANK,
        DRIVERORIENTED
    }
}
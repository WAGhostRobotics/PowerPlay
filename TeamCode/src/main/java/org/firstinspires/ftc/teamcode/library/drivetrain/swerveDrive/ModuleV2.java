package org.firstinspires.ftc.teamcode.library.drivetrain.swerveDrive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.AnalogEncoder;

@Config
public class ModuleV2 {

    private DcMotor motor;
    private CRServo pivot;
    private AnalogEncoder encoder;

    private int motorMultiplier = 1;

    private double targetAngle;

    private double error = 0;
    private double power = 0;

    public HardwareMap.DeviceMapping<VoltageSensor> voltageSensor;

    public static double K_STATIC = 0.1;
    public static double K_STATIC_POS = 0.1;
    public static double K_STATIC_NEG = 0.1;

    public static double voltage = 12.5; //12.08

//    public double K_STATIC = 0.16;

    private final double PERMISSABLE_ERROR = 4;

    public static double p = 0.00185, i = 0.00018, d = 0.02;

//    private double p = 0.00185, i = 0.00018, d = 0.01;

    public PIDController headingController = new PIDController(p, i, d);

    public ModuleV2(DcMotor motor, CRServo pivot, AnalogEncoder encoder){
        this.motor = motor;
        this.pivot = pivot;
        this.encoder = encoder;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public ModuleV2(DcMotor motor, CRServo pivot, AnalogEncoder encoder, double K_STATIC){
        this.motor = motor;
        this.pivot = pivot;
        this.encoder = encoder;
        this.K_STATIC = K_STATIC;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public ModuleV2(DcMotor motor, CRServo pivot, AnalogEncoder encoder, double K_STATIC, double p, double i, double d){
        this.motor = motor;
        this.pivot = pivot;
        this.encoder = encoder;
        this.K_STATIC = K_STATIC;
        this.p = p;
        this.i = i;
        this.d = d;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public ModuleV2(DcMotor motor, CRServo pivot, AnalogEncoder encoder, double K_STATIC, double p, double i, double d, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor){
        this.motor = motor;
        this.pivot = pivot;
        this.encoder = encoder;
        this.K_STATIC = K_STATIC;
        this.p = p;
        this.i = i;
        this.d = d;
//        this.voltageSensor = voltageSensor;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public ModuleV2(DcMotor motor, CRServo pivot, AnalogEncoder encoder, double K_STATIC_POS, double K_STATIC_NEG, double p, double i, double d, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor){
        this.motor = motor;
        this.pivot = pivot;
        this.encoder = encoder;
        this.K_STATIC_POS = K_STATIC_POS;
        this.K_STATIC_NEG = K_STATIC_NEG;
        this.p = p;
        this.i = i;
        this.d = d;
        this.voltageSensor = voltageSensor;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setPower(double power){
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(motorMultiplier * power);
    }

    public void setHold() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setServoPower(double power) {
        pivot.setPower(power);
    }

    public void setTargetAngle(double angle){
        if(normalizeDegrees(angle)!=targetAngle){
//            headingController.reset();
            targetAngle = normalizeDegrees(angle);
        }
    }

    public AnalogEncoder getEncoder(){
        return encoder;
    }

    public double getTargetAngle(){
        return normalizeDegrees(targetAngle);
    }

    public double getModuleAngle(){
        return normalizeDegrees(Math.toDegrees(encoder.getCurrentPosition()));
    }




    public void update(){

        headingController.setPID(p, i, d);

        double target = getTargetAngle();
        double angle = getModuleAngle();
        error = normalizeDegrees(target - angle);

        if(Math.abs(error)>90.0){
            target = normalizeDegrees(target - 180.0);
            motorMultiplier = -1;
        }else {
            motorMultiplier = 1;
        }

        error = normalizeDegrees(target - angle);

        power = Range.clip(headingController.calculate(error), -1, 1);

        // 0-90 overshoot, 90-0 undershoot
        if (Math.signum(power) > 0) {
            power = Math.signum(power) * K_STATIC_POS + power;
        } else {
            power = Math.signum(power) * K_STATIC_NEG + power;
        }

        if(Double.isNaN(power)) power = 0;

        if(Math.abs(error)<=PERMISSABLE_ERROR) {
            power = 0;
        }

        if (voltageSensor == null) {
            pivot.setPower(power);
        }
        else {
            pivot.setPower(power * (voltage / (voltageSensor.iterator().next().getVoltage())));
        }


    }

    public double getPower(){
        return power;
    }

    public double getError(){
        return error;
    }


}

package org.firstinspires.ftc.teamcode.library.drivetrain.swerveDrive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

    public static double K_STATIC = 0.18;

    private final double PERMISSABLE_ERROR = 2;

    public static double p = 0.0015, i = 0, d = 0.001;

    public PIDController headingController = new PIDController(p, i, d);


    public ModuleV2(DcMotor motor, CRServo pivot, AnalogEncoder encoder){
        this.motor = motor;
        this.pivot = pivot;
        this.encoder = encoder;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setPower(double power){
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(motorMultiplier * power);
    }

    public void setServoPower(double power) {
        pivot.setPower(power);
    }

    public void setTargetAngle(double angle){
        if(normalizeDegrees(angle)!=targetAngle){
            headingController.reset();
            targetAngle = normalizeDegrees(angle);
        }
    }

    public AnalogEncoder getEncoder(){
        return encoder;
    }

    public double getTargetAngle(){
        return normalizeDegrees(targetAngle);
    }

    public double  getModuleAngle(){
        return normalizeDegrees(Math.toDegrees(encoder.getCurrentPosition()));
    }




    public void update(){

        headingController.setPID(p, i, d);

        double target = getTargetAngle();
        double angle = getModuleAngle();
        error = normalizeDegrees(target - angle);

        if(Math.abs(error)>90.0){
            target = normalizeDegrees(target - 180.0);
            motorMultiplier *= -1;
        }

        error = normalizeDegrees(target - angle);

        power = Range.clip(headingController.calculate(0, error), -1, 1);

        power = Math.signum(power) * K_STATIC + power;
        if(Double.isNaN(power)) power = 0;

        if(Math.abs(error)<=PERMISSABLE_ERROR) {
            power = 0;
        }

        pivot.setPower(power);


    }

    public double getPower(){
        return power;
    }

    public double getError(){
        return error;
    }


}

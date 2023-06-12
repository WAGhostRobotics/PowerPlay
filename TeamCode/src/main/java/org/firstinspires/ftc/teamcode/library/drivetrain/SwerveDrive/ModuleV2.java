package org.firstinspires.ftc.teamcode.library.drivetrain.SwerveDrive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.AnalogEncoder;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class ModuleV2 {

    private DcMotor motor;
    private CRServo pivot;
    private AnalogEncoder encoder;

    private int motorMultiplier = 1;

    private double targetAngle;

    private final int TICKS_PER_REV = 8192;

    private PIDController headingController;



    public ModuleV2(HardwareMap hwMap, String motorName, String servoName, String encoderName, boolean reset){
        motor = hwMap.get(DcMotor.class, motorName);
        pivot = hwMap.get(CRServo.class, servoName);
        encoder = new AnalogEncoder(hwMap.get(AnalogInput.class, encoderName));

        if(reset){
            encoder.zero();
        }

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        headingController = new PIDController(0,0,0);

    }

    public void setPower(double power){
        motor.setPower(motorMultiplier * Math.abs(power));
    }

    public void setTargetAngle(double angle){
        targetAngle = normalizeDegrees(angle);
    }

    public double getTargetAngle(){
        return normalizeDegrees(targetAngle - 180.0);
    }

    public double getModuleAngle(){
        return normalizeDegrees(Math.toDegrees(encoder.getCurrentPosition())-180);
    }



    public void update(){
        double target = getTargetAngle();
        double angle = getModuleAngle();
        double error = normalizeDegrees(target - angle);

        if(Math.abs(error)>90){
            target = normalizeDegrees(target - 180);
            motorMultiplier *= 1;
        }

        error = normalizeDegrees(target - angle);

        double power = Range.clip(headingController.calculate(0, error), -1, 1);
        if(Double.isNaN(power)) power = 0;

        pivot.setPower(power);


    }










}

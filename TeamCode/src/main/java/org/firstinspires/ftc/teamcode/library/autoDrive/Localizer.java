package org.firstinspires.ftc.teamcode.library.autoDrive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class Localizer {

    private Encoder rightEncoder;
    private Encoder frontEncoder;
    private Encoder leftEncoder;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1   ; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 0; // in; distance between the left and right wheels 4.125
    public static double FORWARD_OFFSET = 0; // in; offset of the lateral wheel
//public static double FORWARD_OFFSET = -1.1811; // in; offset of the lateral wheel

    double x;
    double y;


    double lastHeading;
    double lastX;
    double lastY;

    double rawX;
    double rawY;
    double heading;

    double r0;
    double r1;


    double relX;
    double relY;

    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction


    public Localizer(){
        reset();
    }

    public Localizer(HardwareMap hardwareMap){

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rf"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rr"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lf"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

//        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        rightEncoder.reset();
        frontEncoder.reset();
        leftEncoder.reset();

        reset();

    }

    public void reset(){
        x = 0;
        y = 0;

        lastHeading = 0;
        lastX = 0;
        lastY = 0;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public void update(){

        calculateRawValues();

        if(heading -lastHeading == 0){
            relX = (rawX -lastX);
            relY = (rawY -lastY);
        }else{
            r0 = (rawX -lastX) / (heading - lastHeading);
            r1 = (rawY -lastY) / (heading - lastHeading);

            relX = r0 * Math.sin(heading -lastHeading) - r1 * (1 - Math.cos(heading -lastHeading));
            relY = r1 * Math.sin(heading -lastHeading) + r0 * (1 - Math.cos(heading -lastHeading));
        }

        x += relX * Math.cos(heading) - relY * Math.sin(heading);
        y += relY * Math.cos(heading) + relX * Math.sin(heading);

        lastX = rawX;
        lastY = rawY;
        lastHeading = heading;

    }

    public void calculateRawValues(){
        double rawX = (getRightEncoderPosition() + getLeftEncoderPosition())/2.0;
        double heading = (getRightEncoderPosition() - getLeftEncoderPosition())/(LATERAL_DISTANCE);
        double rawY = getFwdEncoderPosition() - (FORWARD_OFFSET * heading);

        setRawValues(rawX, rawY, heading);
    }

    public double getLastHeading(){
        return lastHeading;
    }

    public void setRawValues(double x, double y, double head){
        rawX = x;
        rawY = y;
        heading = head;
    }


    public double getHeading(Angle angle){
        if(angle == Angle.RADIANS){
            return getHeading();
        }else{
            return Math.toDegrees(getHeading());

        }
    }

    public double getHeading(){
        return heading;
    }

    public double getRawX(){
        return rawX;
    }

    public double getRawY(){
        return rawY;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    private double getRightEncoderPosition(){
        return encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER;
    }

    private double getLeftEncoderPosition(){
        return encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER;
    }

    private double getFwdEncoderPosition(){
        return encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER;
    }


    public enum Angle{
        RADIANS,
        DEGREES,
    }
}
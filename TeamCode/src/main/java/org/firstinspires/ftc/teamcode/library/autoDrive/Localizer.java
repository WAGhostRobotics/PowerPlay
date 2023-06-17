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

    public static double LATERAL_DISTANCE = 14.125; // in; distance between the left and right wheels 4.125
    public static double FORWARD_OFFSET = -3.72; // in; offset of the lateral wheel

    double x;
    double y;


    double lastHeading;
    double lastX;
    double lastY;

    double currentX;
    double currentY;
    double currentHeading;

    double r0;
    double r1;

    double dtheta;

    double relX;
    double relY;

    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction


    public Localizer(HardwareMap hardwareMap){

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rf"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lf"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        rightEncoder.reset();
        frontEncoder.reset();
        leftEncoder.reset();

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

        currentX = getRawX();
        currentY = getRawY();
        currentHeading = getHeading();

        if(currentHeading-lastHeading == 0){
            relX = (currentX-lastX);
            relY = (currentY-lastY);
        }else{
            r0 = (currentX-lastX) / (currentHeading - lastHeading);
            r1 = (currentY-lastY) / (currentHeading - lastHeading);

            relX = r0 * Math.sin(currentHeading-lastHeading) - r1 * (1 - Math.cos(currentHeading-lastHeading));
            relY = r1 * Math.sin(currentHeading-lastHeading) + r0 * (1 - Math.cos(currentHeading-lastHeading));
        }

        x += relX * Math.cos(currentHeading) - relY * Math.sin(currentHeading);
        y += relY * Math.cos(currentHeading) + relX * Math.sin(currentHeading);

        lastX = currentX;
        lastY = currentY;
        lastHeading = currentHeading;

    }



    public double getHeading(Angle angle){
        if(angle == Angle.RADIANS){
            return getHeading();
        }else{
            return Math.toDegrees(getHeading());

        }
    }

    public double getHeading(){
        return (getRightEncoderPosition() - getLeftEncoderPosition())/(LATERAL_DISTANCE);
    }

    public double getRawX(){
        return (getRightEncoderPosition() + getLeftEncoderPosition())/2.0;
    }

    public double getRawY(){
        return getFwdEncoderPosition() - (FORWARD_OFFSET * getHeading());
    }

    public String getRawEncoders(){
        return "" + rightEncoder.getCurrentPosition() + " " + leftEncoder.getCurrentPosition() + " " + frontEncoder.getCurrentPosition();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public double getRightEncoderPosition(){
        return encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER;
    }

    public double getLeftEncoderPosition(){
        return encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER;
    }

    public double getFwdEncoderPosition(){
        return encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER;
    }


    public enum Angle{
        RADIANS,
        DEGREES,
    }
}

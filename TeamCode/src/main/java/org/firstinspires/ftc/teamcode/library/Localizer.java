package org.firstinspires.ftc.teamcode.library;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class Localizer {

    private Encoder xEncoderRight;
    private Encoder yEncoder;
    private Encoder xEncoderLeft;

    public static double WHEEL_RADIUS = 96.0/2/25.4 ; // in
    public static double GEAR_RATIO = 1 ; // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 537.6;

    public static double LATERAL_DISTANCE = 14.125; // in; distance between the left and right wheels 4.125
    public static double FORWARD_OFFSET = -3.8; // in; offset of the lateral wheel

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


    public Localizer(Encoder xEncoderRight,
                     Encoder yEncoder, Encoder xEncoderLeft){

        this.xEncoderRight = xEncoderRight;
        this.yEncoder = yEncoder;
        this.xEncoderLeft = xEncoderLeft;

        xEncoderRight.reset();
        yEncoder.reset();
        xEncoderLeft.reset();

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

    public double getHeading(){
        return (xEncoderRight.getCurrentPosition() - xEncoderLeft.getCurrentPosition())/(LATERAL_DISTANCE);
    }

    public double getHeading(Angle angle){
        if(angle == Angle.RADIANS){
            return (xEncoderRight.getCurrentPosition() - xEncoderLeft.getCurrentPosition())/(LATERAL_DISTANCE);
        }else{
            return (180/Math.PI)*(xEncoderRight.getCurrentPosition() - xEncoderLeft.getCurrentPosition())/(LATERAL_DISTANCE);

        }
    }


    private double getRawX(){
        return encoderTicksToInches((xEncoderRight.getCurrentPosition() + xEncoderLeft.getCurrentPosition())/2.0);
    }

    private double getRawY(){
        return encoderTicksToInches(yEncoder.getCurrentPosition()) - (FORWARD_OFFSET * getHeading());
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }


    public enum Angle{
        RADIANS,
        DEGREES,
    }
}

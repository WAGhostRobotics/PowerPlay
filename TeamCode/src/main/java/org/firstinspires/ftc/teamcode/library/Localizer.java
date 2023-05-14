package org.firstinspires.ftc.teamcode.library;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class Localizer {

    private Encoder xEncoder;
    private Encoder yEncoder;
    private Encoder x2Encoder;

    public static double WHEEL_RADIUS = 96.0/2/25.4 ; // in
    public static double GEAR_RATIO = 1 ; // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 537.6;

    public static double LATERAL_DISTANCE = 14.125; // in; distance between the left and right wheels 4.125
    public static double FORWARD_OFFSET = -3.8; // in; offset of the lateral wheel

    public Localizer(Encoder xEncoder,
                     Encoder yEncoder, Encoder x2Encoder){

        this.xEncoder = xEncoder;
        this.yEncoder = yEncoder;
        this.x2Encoder = x2Encoder;

    }


    public double getX(){

        return 0;
    }

    public double getY(){

        return 0;
    }

    public double getHeading(){
        return 0;
    }

    public void update(){

    }

    private double getRawX(){
        return 0;
    }

    private double getRawY(){
        return 0;
    }
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}

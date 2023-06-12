package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogEncoder {

    private final static double RANGE = 3.3;
    private final static boolean VALUE_REJECTION = false;

    private boolean reverse;

    private double pastPosition = 1;

    private AnalogInput encoder;
    private double zero;
    private double range;



    public AnalogEncoder(AnalogInput encoder, double range){
        this.encoder = encoder;
        this.range = range;
        this.zero = 0;
        this.reverse = false;
    }

    public AnalogEncoder(AnalogInput encoder){
        this(encoder, RANGE);
    }
    public void zero(){
        zero = getCurrentPosition();
    }
    public void setInverted(boolean invert){
        reverse = invert;
    }
    public boolean getDirection() {
        return reverse;
    }

    public double getCurrentPosition() {
        double pos = Angle.norm((!reverse ? 1 - getVoltage() / range : getVoltage() / range) * Math.PI*2 - zero);

        if(!VALUE_REJECTION || Math.abs(Angle.normDelta(pastPosition)) > 0.1 || Math.abs(Angle.normDelta(pos)) < 1) pastPosition = pos;
        return pastPosition;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public static double reLinearize(double v){
        double v2 = 3.3-v;
        double alpha = v2 * ((3.3 * v2) - 16.335);
        double beta = (v2 * v2) - (3.3 * v2) - 5.445;
        return 3.3 - (alpha / beta);
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }
}

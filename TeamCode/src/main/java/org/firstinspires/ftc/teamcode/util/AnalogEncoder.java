package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogEncoder {

    private final static double RANGE = 3.3;

    private boolean reverse;

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
    public void reverse(boolean reverse){
        this.reverse = reverse;
    }
    public boolean getDirection() {
        return reverse;
    }

    public double getCurrentPosition() {
        double pos;

        if(reverse){
            pos = Angle.norm((1 - getVoltage() / range) * 2 * Math.PI - zero);
        }else{
            pos = Angle.norm((getVoltage() / range) * 2 * Math.PI - zero);
        }

        return pos;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }
}

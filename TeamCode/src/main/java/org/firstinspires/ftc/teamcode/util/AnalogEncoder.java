package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogEncoder {

    private final static double RANGE = 0.294;

    private boolean reverse = false;

    private AnalogInput encoder;
    private double zero = 0;
    private double range = RANGE;
    private double last = 0;



    public AnalogEncoder(AnalogInput encoder, double zero, double range) {
        this.encoder = encoder;
        this.range = range;
        this.zero = zero;
    }

    public AnalogEncoder(AnalogInput encoder, double zero, double range, boolean reverse){
        this.encoder = encoder;
        this.range = range;
        this.zero = zero;
        this.reverse = reverse;
    }

    public AnalogEncoder(AnalogInput encoder, double zero){
        this.encoder = encoder;
        this.zero = zero;

    }

    public AnalogEncoder(AnalogInput encoder){
        this.encoder = encoder;
    }

    public AnalogEncoder(AnalogInput encoder, double zero, boolean reverse) {
        this.encoder = encoder;
        this.zero = zero;
        this.reverse = reverse;
    }




    public void zero(){
        zero = getCurrentPosition();
    }
    public double getZero(){
        return zero;
    }
    public void setZero(double zero){
        this.zero = zero;
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
            pos = Angle.norm((1 - getVoltage() / range) * 2 * Math.PI + Math.toRadians(zero));
        }else{
            pos = Angle.norm((getVoltage() / range) * 2 * Math.PI - Math.toRadians(zero));
        }

// nakul's bro logic
        if(Math.abs(normalizeRadians(last+Math.toRadians(zero)))>0.2||Math.abs(normalizeRadians(pos+Math.toRadians(zero)))<1.0){
            last=pos;
            return pos;
        }else {
            return last;
        }


    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }
}

package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogEncoder {


    private boolean reverse = false;

    private AnalogInput encoder;
    private double zero = 0;
    private double range = 3.385;
    private double last=1;




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


        double offset = -Math.toRadians(zero);


        pos = Angle.norm(((reverse?(1- getVoltage()/range):(getVoltage()/range))) * 2 * Math.PI + offset);

// nakul's bro logic
        if(Math.abs(normalizeRadians(last-offset))>0.2 || Math.abs(normalizeRadians(pos-offset))<1.0){
            last=pos;
            return pos;
        }else {
            return last;
        }
//        return pos;


    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }
}

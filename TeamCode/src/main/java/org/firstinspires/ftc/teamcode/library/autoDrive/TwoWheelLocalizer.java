package org.firstinspires.ftc.teamcode.library.autoDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.component.Imu;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class TwoWheelLocalizer extends Localizer {

    private Encoder parallelEncoder;
    private Encoder perpendicularEncoder;
    private Imu imu;


    public static double PERPENDICULAR_X = 0;
    public static double PARALLEL_Y = 0;


    public TwoWheelLocalizer(LinearOpMode opMode, HardwareMap hardwareMap){

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rf"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rr"));
        imu = new Imu(hardwareMap);
        imu.initImuThread(opMode);

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)


        parallelEncoder.reset();
        perpendicularEncoder.reset();

    }

    @Override
    public void calculateRawValues(){
        double headingChange = Math.toRadians(imu.getCurrentHeading()) - getLastHeading();
        if(headingChange > 180){
            headingChange -= 360;
        }else if(headingChange<-180){
            headingChange += 360;
        }

        double heading = getLastHeading() + headingChange;
        double rawX = getParallelEncoderPosition() - (PARALLEL_Y * heading);
        double rawY = getPerpendicularEncoderPosition() - (PERPENDICULAR_X * heading);

        setRawValues(rawX, rawY, heading);
    }

    private double getParallelEncoderPosition(){
        return encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER;
    }

    private double getPerpendicularEncoderPosition(){
        return encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER;
    }


}

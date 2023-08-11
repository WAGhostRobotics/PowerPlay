package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.function.DoubleSupplier;

import javax.annotation.concurrent.GuardedBy;

public class Imu {

    Thread imuThread;
    private final Object imuSync = new Object();

    @GuardedBy("imuSync")
    private BNO055IMU imu;
    private double heading = 0;
    private double angularVelocity = 0;

    HardwareMap hwMap;

    public Imu(HardwareMap hwMap){
        this.hwMap = hwMap;
        initIMU();
    }

    //gets angle from imu
    public double getCurrentHeading() {
        return heading;
    }

    public void initImuThread(LinearOpMode opMode){
        imuThread = new Thread(() -> {
            synchronized (imuSync){
                while (!opMode.isStarted() && !opMode.isStopRequested()) {

                }
                while(opMode.opModeIsActive() && !opMode.isStopRequested()){
                    heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    angularVelocity = imu.getAngularVelocity().zRotationRate;
                }

            }
        });
        imuThread.start();
    }

    public BNO055IMU getImu(){
        return imu;
    }


    public double getAngularVelocity(){
        return angularVelocity;
    }


    public void initIMU(){
        synchronized (imuSync){
            imu = hwMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            imu.initialize(parameters);
        }

    }

}

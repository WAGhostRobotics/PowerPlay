package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Imu {

    private BNO055IMU imu;
    HardwareMap hwMap;
    Thread imuThread;
    Object imuSync = new Object();

    private double heading;
    private double angularVelocity;

    public Imu(HardwareMap hwMap){
        this.hwMap = hwMap;

        imu = hwMap.get(BNO055IMU.class, "imu");
        initIMU();
        heading = 0;
        angularVelocity = 0;
    }

    //gets angle from imu
    public double getCurrentHeading() {
        return heading;
    }

    public void initImuThread(){
        imuThread = new Thread(() -> {
            synchronized (imuSync){
                try{
                    heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    angularVelocity = imu.getAngularVelocity().xRotationRate;
                }catch (Exception e){
                    //oops
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
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

}

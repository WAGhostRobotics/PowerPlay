package org.firstinspires.ftc.teamcode.library;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.math.Bezier;
import org.firstinspires.ftc.teamcode.library.math.Point;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Drive {

    private Bezier spline;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private Encoder xEncoder;
    private Encoder yEncoder;
    private Encoder y2Encoder;
    private double heading;


    private PIDController xControl;
    private PIDController yControl;

    private PIDController headingControl;


    private PIDController xControlEnd;
    private PIDController yControlEnd;

    private PIDController headingControlEnd;

    public static double WHEEL_RADIUS = 96.0/2/25.4 ; // in
    public static double GEAR_RATIO = 1 ; // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 537.6;


    private double t1, t2, t3, time;

    public static final int MAX_RPM = 349;
    public static double MAX_VEL = (MAX_RPM/60.0) * (GEAR_RATIO) * (WHEEL_RADIUS) * (2* Math.PI) * 1.2; // was * 0.9
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = 4.5601312058986245;
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

    private double t;
    private double x;
    private double y;
    private double theta;
    private double sin;
    private double cos;
    private double maxMovement;
    private double magnitude;
    private double driveTurn;
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private double temp_x;
    private double temp_y;
    private double currentHeading;

    private final double movementPower = 0.6;
    private final double y_error = 1;
    private final double heading_error = 8;
    private final double x_error = 1;

    private ElapsedTime timer;

    public Drive(DcMotor frontLeft, DcMotor frontRight,
                 DcMotor backRight, DcMotor backLeft, Encoder xEncoder,
                 Encoder yEncoder, Encoder y2Encoder){

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.xEncoder = xEncoder;
        this.yEncoder = yEncoder;
        this.y2Encoder = y2Encoder;



    }

    public void startTrajectory(Bezier spline, double heading) {
        this.spline = spline;
        this.heading = heading;

        timer = new ElapsedTime();

        t1 = MAX_VEL/MAX_ACCEL; // time to accelerate
        t3 = MAX_VEL/MAX_ACCEL; // time to decelerate

        t2 = (spline.approximateLength() - 2 * (0.5 * MAX_ACCEL * Math.pow(t1, 2)))/MAX_VEL - t1 - t3; // time in da middle

        time = t1 + t2 + t3;

        xControl = new PIDController(0,0,0);
        yControl = new PIDController(0,0,0);
        headingControl = new PIDController(0,0,0);

        xControlEnd = null;
        yControlEnd = null;
        headingControlEnd = null;

    }



    public void update() {
        t = timer.seconds()/time;

        Point pointWhereItShouldBe = spline.getPoint(t);

        x = encoderTicksToInches(xEncoder.getCurrentPosition());
        y = encoderTicksToInches(yEncoder.getCurrentPosition());
        currentHeading = 0;
        driveTurn = 0;


        if(!((Math.abs(spline.getEndPoint().getX()-x)<= x_error)
                &&(Math.abs(spline.getEndPoint().getY()-y)<= y_error)
                &&(Math.abs(heading-currentHeading)<= heading_error))){

            if(xControlEnd == null || yControlEnd == null || headingControlEnd == null){
                xControlEnd = new PIDController(0,0,0);
                yControlEnd = new PIDController(0,0,0);
                headingControlEnd = new PIDController(0,0,0);
            }

            if(Math.hypot(spline.getEndPoint().getX()-x, spline.getEndPoint().getY()-y) <
                    Math.hypot(encoderTicksToInches(xEncoder.getCorrectedVelocity()), encoderTicksToInches(yEncoder.getCorrectedVelocity()))/(2*MAX_ACCEL)){

                temp_x = xControlEnd.calculate(0, spline.getEndPoint().getX()-x);
                temp_y = yControlEnd.calculate(0, spline.getEndPoint().getY()-y);

                magnitude = Math.hypot(temp_x, temp_y);
                theta = Math.toDegrees(Math.atan2(temp_y, temp_x))-45;

                //sin and cos of robot movement
                sin = Math.sin(Math.toRadians(theta));
                cos = Math.cos(Math.toRadians(theta));
                maxMovement = Math.max(Math.abs(sin), Math.abs(cos));

                //determines powers and scales based on max movement
                frontLeftPower = (magnitude * cos / maxMovement + driveTurn);
                frontRightPower = (magnitude * sin / maxMovement - driveTurn);
                backLeftPower = (magnitude * sin / maxMovement + driveTurn);
                backRightPower = (magnitude * cos / maxMovement - driveTurn);

                //scales if -1 < powers < 1
                if(magnitude + Math.abs(driveTurn)>1){
                    frontLeftPower /= magnitude + driveTurn;
                    frontRightPower /= magnitude + driveTurn;
                    backLeftPower /= magnitude + driveTurn;
                    backRightPower /= magnitude + driveTurn;
                }

                //sets powers scaled to desired speed
                frontLeft.setPower(movementPower*frontLeftPower);
                frontRight.setPower(movementPower*frontRightPower);
                backLeft.setPower(movementPower*backLeftPower);
                backRight.setPower(movementPower*backRightPower);

            } else {

                magnitude = 1;
                theta = Math.toDegrees(Math.atan2(pointWhereItShouldBe.getY(), pointWhereItShouldBe.getX()));

                temp_x = magnitude * Math.cos(Math.toRadians(theta)) + xControl.calculate(x, pointWhereItShouldBe.getX());
                temp_y = magnitude * Math.sin(Math.toRadians(theta)) + yControl.calculate(y, pointWhereItShouldBe.getY());


                magnitude = Math.hypot(temp_x, temp_y);
                theta = Math.toDegrees(Math.atan2(temp_y, temp_x))-45;


                //sin and cos of robot movement
                sin = Math.sin(Math.toRadians(theta));
                cos = Math.cos(Math.toRadians(theta));
                maxMovement = Math.max(Math.abs(sin), Math.abs(cos));

                //determines powers and scales based on max movement
                frontLeftPower = (magnitude * cos / maxMovement + driveTurn);
                frontRightPower = (magnitude * sin / maxMovement - driveTurn);
                backLeftPower = (magnitude * sin / maxMovement + driveTurn);
                backRightPower = (magnitude * cos / maxMovement - driveTurn);

                //scales if -1 < powers < 1
                if(magnitude + Math.abs(driveTurn)>1){
                    frontLeftPower /= magnitude + driveTurn;
                    frontRightPower /= magnitude + driveTurn;
                    backLeftPower /= magnitude + driveTurn;
                    backRightPower /= magnitude + driveTurn;
                }

                double max = Math.max(frontLeftPower, Math.max(frontRightPower, Math.max(backLeftPower, backRightPower)));

                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;


                //sets powers scaled to desired speed
                frontLeft.setPower(movementPower*frontLeftPower);
                frontRight.setPower(movementPower*frontRightPower);
                backLeft.setPower(movementPower*backLeftPower);
                backRight.setPower(movementPower*backRightPower);
            }




        }
    }


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}

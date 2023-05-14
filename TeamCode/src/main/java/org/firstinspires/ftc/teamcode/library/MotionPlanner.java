package org.firstinspires.ftc.teamcode.library;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.math.Bezier;
import org.firstinspires.ftc.teamcode.library.math.Point;

public class MotionPlanner {

    private Bezier spline;
    private double heading;

    private Drive drive;
    private Localizer localizer;

    private PIDController xControl;
    private PIDController yControl;

    private PIDController headingControl;


    private PIDController xControlEnd;
    private PIDController yControlEnd;

    private PIDController headingControlEnd;

    private double t1, t2, t3, time;


    private double t;
    private double x;
    private double y;
    private double theta;
    private double magnitude;
    private double driveTurn;
    private double x_power;
    private double y_power;
    private double x_rotated;
    private double y_rotated;
    private double currentHeading;

    public static double WHEEL_RADIUS = 96.0/2/25.4 ; // in
    public static double GEAR_RATIO = 1 ; // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 537.6;

    public static final int MAX_RPM = 349;
    public static double MAX_VEL = (MAX_RPM/60.0) * (GEAR_RATIO) * (WHEEL_RADIUS) * (2* Math.PI) * 1.2; // was * 0.9
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = 4.5601312058986245;
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

    private final double movementPower = 0.6;
    private final double translational_error = 1;
    private final double heading_error = 8;

    private ElapsedTime timer;

    public MotionPlanner(Drive drive, Localizer localizer){

        this.drive = drive;

        this.localizer = localizer;


    }

    public void startTrajectory(Bezier spline, double heading) {
        this.spline = spline;
        this.heading = heading;

        timer = new ElapsedTime();

        t1 = MAX_VEL/MAX_ACCEL; // time to accelerate
        t3 = MAX_VEL/MAX_ACCEL; // time to decelerate

        t2 = (spline.approximateLength() - 2 * (0.5 * MAX_ACCEL * Math.pow(t1, 2)))/MAX_VEL - t1 - t3; // time in da middle


        if(t2<0){
            t2 = 0;
        }
        time = t1 + t2 + t3;

        xControl = new PIDController(0,0,0);
        yControl = new PIDController(0,0,0);
        headingControl = new PIDController(0,0,0);

        xControlEnd = null;
        yControlEnd = null;
        headingControlEnd = null;

    }



    public void update() {

        localizer.update();

        t = timer.seconds()/time;

        Point pointWhereItShouldBe = spline.getPoint(t);
        Point derivative = spline.getDerivative(t);

        x = localizer.getX();
        y = localizer.getY();
        currentHeading = localizer.getHeading(Localizer.Angle.DEGREES);


        if(!((Math.hypot(spline.getEndPoint().getX()-x, spline.getEndPoint().getY()-y)< translational_error)
                &&(Math.abs(heading-currentHeading)<= heading_error))){


            if((Math.hypot(spline.getEndPoint().getX()-x, spline.getEndPoint().getY()-y) <
                    Math.hypot(localizer.getX(), localizer.getY())/(2*MAX_ACCEL))||t>=1){

                if(xControlEnd == null || yControlEnd == null || headingControlEnd == null){
                    xControlEnd = new PIDController(0,0,0);
                    yControlEnd = new PIDController(0,0,0);
                    headingControlEnd = new PIDController(0,0,0);
                }

                x_power = xControlEnd.calculate(0, spline.getEndPoint().getX()-x);
                y_power = yControlEnd.calculate(0, spline.getEndPoint().getY()-y);

                x_rotated = x_power * Math.cos(Math.toRadians(heading)) - y_power * Math.sin(Math.toRadians(heading));
                y_rotated = x_power * Math.sin(Math.toRadians(heading)) + y_power * Math.cos(Math.toRadians(heading));


                magnitude = Math.hypot(x_rotated, y_rotated);
                theta = Math.toDegrees(Math.atan2(y_rotated, x_rotated));
                driveTurn = headingControl.calculate(currentHeading, heading);


                drive.drive(magnitude, theta, driveTurn, movementPower);

                //sets powers scaled to desired speed

            } else {

                magnitude = 1;
                theta = Math.toDegrees(Math.atan2(derivative.getY(), derivative.getX()));

                x_power = magnitude * Math.cos(Math.toRadians(theta)) + xControl.calculate(x, pointWhereItShouldBe.getX());
                y_power = magnitude * Math.sin(Math.toRadians(theta)) + yControl.calculate(y, pointWhereItShouldBe.getY());

                x_rotated = x_power * Math.cos(Math.toRadians(heading)) - y_power * Math.sin(Math.toRadians(heading));
                y_rotated = x_power * Math.sin(Math.toRadians(heading)) + y_power * Math.cos(Math.toRadians(heading));

                magnitude = Math.hypot(x_rotated, y_rotated);
                theta = Math.toDegrees(Math.atan2(y_rotated, x_rotated));
                driveTurn = headingControl.calculate(currentHeading, heading);

                drive.driveMax(magnitude, theta, driveTurn, movementPower);
            }

        }
    }
}

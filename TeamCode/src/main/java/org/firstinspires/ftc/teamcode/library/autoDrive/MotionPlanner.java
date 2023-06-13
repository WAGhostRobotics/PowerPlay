package org.firstinspires.ftc.teamcode.library.autoDrive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;

public class MotionPlanner {

    private Bezier spline;
    private double heading;

    private Drivetrain drive;
    private Localizer localizer;

//    private PIDController translationalControl = new PIDController(0.022,0.001,0.03);
    private PIDController translationalControl = new PIDController(1.2,0.08,0.15);
    private PIDController headingControl = new PIDController(0.01, 0.008, 0.005);

//    private PIDController translationalControlEnd = new PIDController(0.022,0.001,0.03);
    private PIDController translationalControlEnd = new PIDController(0.4,0.01,0.03);
    private PIDController headingControlEnd = new PIDController(0.04, 0.02, 0.005);


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


    double y1;
    double y2;

    private double velocity;

    double lasty;
    double lasty1;
    double lastx;

    double currentY;
    double currentX;

    double radius;
    public final double THE_HOLY_CONSTANT = 0.001; //0.01

    double ac;

    double numLoops;
    ElapsedTime loopTime;


    public static double WHEEL_RADIUS = 96.0/2/25.4 ; // in
    public static double GEAR_RATIO = 1 ; // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 537.6;

    public static final int MAX_RPM = 349;
//    public static double MAX_VEL = (MAX_RPM/60.0) * (GEAR_RATIO) * (WHEEL_RADIUS) * (2* Math.PI) * 1.5; // was * 0.9
public static double MAX_VEL = 42.22; // was * 0.9
    public static double MAX_ACCEL = 85;
    public static double MAX_ANG_VEL = 4.5601312058986245;
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

    private final double movementPower = 0.7;
    private final double translational_error = 0.5;
    private final double heading_error = 1;

    private final double endTrajThreshhold = 6;

    boolean end = false;

    private ElapsedTime timer;
    private ElapsedTime ACtimer;

    private double distanceLeft;
    private double estimatedStopping;

    Point target;
    Point derivative;

    double perpendicularError;


    public MotionPlanner(MecanumDrive drive, Localizer localizer){

        this.drive = drive;
        this.localizer = localizer;


    }

    public void startTrajectory(Bezier spline, double heading) {
        this.spline = spline;
        this.heading = heading;

        timer = new ElapsedTime();
        ACtimer = new ElapsedTime();

        t1 = MAX_VEL/MAX_ACCEL; // time to accelerate
        t3 = MAX_VEL/MAX_ACCEL; // time to decelerate

        t2 = (spline.approximateLength() - 2 * (0.5 * MAX_ACCEL * Math.pow(t1, 2)))/MAX_VEL; // time in da middle


        if(t2<0){
            t2 = 0;
        }
        time = t1 + t2 + t3;


        translationalControl.reset();
        headingControl.reset();

        numLoops = 0;
        loopTime = new ElapsedTime();

    }

    public String getTelemetry(){
        return "Time: " + time +
                "\n Theta: " + theta +
                "\n Magnitude: " + magnitude +
                "\n Phase: " + end +
                "\n Stop " + (distanceLeft < estimatedStopping) +
                "\n Distance left: " + distanceLeft +
//                "\n Distance left (x): " + (spline.getEndPoint().getX()-x) +
//                "\n Distance left (y): " + (spline.getEndPoint().getY()-y) +
                "\n Perpendicular error: " + (perpendicularError) +
                "\n Heading: " + (heading - currentHeading) +
                "\n Estimated Stopping " + estimatedStopping +
                "\n " + drive.getTelemetry() +
                "\n Finished " + isFinished()+
                "\n Loop Rate " + numLoops/loopTime.seconds();
    }



    public void update() {

        localizer.update();
        updateACValues();

        t = timer.seconds()/time;

        target = spline.getPoint(t);
        derivative = spline.getDerivative(t);

        x = localizer.getX();
        y = localizer.getY();
        currentHeading = localizer.getHeading(Localizer.Angle.DEGREES);


        if(!isFinished()){

            distanceLeft = Math.hypot(spline.getEndPoint().getX()-x, spline.getEndPoint().getY()-y);
            estimatedStopping = Math.abs(velocity)/(2*MAX_ACCEL);
            //distanceLeft < estimatedStopping

            if(distanceLeft <= endTrajThreshhold||t>=1){



                if(!end){
                    translationalControlEnd.reset();
                    headingControlEnd.reset();
                }

                end = true;


                x_power = translationalControlEnd.calculate(0, spline.getEndPoint().getX()-x);
                y_power = translationalControlEnd.calculate(0, spline.getEndPoint().getY()-y);

                x_rotated = x_power * Math.cos(Math.toRadians(currentHeading)) + y_power * Math.sin(Math.toRadians(currentHeading));
                y_rotated =  -x_power * Math.sin(Math.toRadians(currentHeading)) + y_power * Math.cos(Math.toRadians(currentHeading));

                magnitude = Math.hypot(x_rotated, y_rotated);
                theta = Math.toDegrees(Math.atan2(y_rotated, x_rotated));
                driveTurn = headingControlEnd.calculate(0, heading - currentHeading);



                drive.drive(magnitude, theta, driveTurn, movementPower);


            } else {

                end = false;

                magnitude = 1;

                double vy = derivative.getY();
                double vx = derivative.getX();

                theta = Math.toDegrees(Math.atan2(vy, vx));


                /********DUMB*********/

                double correction;
                if(vx == 0){
                    perpendicularError = target.getX() - x;

                    correction = translationalControl.calculate(0, perpendicularError);
                    theta -= Math.toDegrees(Math.atan2(correction, magnitude));

                }else{
                    double slope = vy/vx;
                    double yIntTarget = (target.getY() - (slope)*(target.getX()));
                    double yIntReal = (y - (slope)*x);

                    perpendicularError = Math.abs(yIntTarget-yIntReal)/Math.sqrt(1 + Math.pow(slope, 2));
                    correction = translationalControl.calculate(0, perpendicularError);

                    double multiplier = 1;

                    if(normalizeDegrees(theta-90)>0){
                        multiplier = -1;
                    }

                    if(yIntTarget <= yIntReal){
                        theta -= multiplier * Math.toDegrees(Math.atan2(correction, magnitude));
                    }else{
                        theta += multiplier * Math.toDegrees(Math.atan2(correction, magnitude));
                    }

                }
                magnitude = Math.hypot(magnitude, correction);


                theta += Math.toDegrees(Math.atan2(correction, magnitude));
                magnitude = Math.hypot(magnitude, correction);

                x_power = magnitude * Math.cos(Math.toRadians(theta));
                y_power = magnitude * Math.sin(Math.toRadians(theta));



                x_rotated = x_power * Math.cos(Math.toRadians(currentHeading)) + y_power * Math.sin(Math.toRadians(currentHeading));
                y_rotated = -x_power * Math.sin(Math.toRadians(currentHeading)) + y_power * Math.cos(Math.toRadians(currentHeading));

                magnitude = Math.hypot(x_rotated, y_rotated);
                theta = Math.toDegrees(Math.atan2(y_rotated, x_rotated));
                driveTurn = headingControl.calculate(0, heading - currentHeading);

                if(!Double.isNaN(y1)&&!Double.isNaN(y2) && magnitude != 0){
                    radius = Math.pow((1+Math.pow(y1,2)), 1.5)/y2;
                    ac = Math.pow(velocity, 2)/radius;
                    theta -= Math.toDegrees(Math.atan2( ac*THE_HOLY_CONSTANT, magnitude));
                    magnitude = Math.hypot(magnitude, ac*THE_HOLY_CONSTANT);

                }else{
                    ac = 0;
                }

                drive.driveMax(magnitude, theta, driveTurn, movementPower);
            }

        }else{
            drive.drive(0, 0, 0, 0);
        }

        numLoops++;
    }

    public void updateACValues(){
        currentX = localizer.getRawY();
        currentY = localizer.getRawX();

        if((currentX-lastx) == 0){
            y1 = Double.NaN;
            y2 = Double.NaN;
        }else{
            y1 = (currentY-lasty)/(currentX-lastx);
            y2 = (y1-lasty1)/(currentX-lastx);
        }


        double ACtime = ACtimer.seconds();

        velocity = Math.sqrt(
                Math.pow(((currentX-lastx)/ACtime), 2) + Math.pow(((currentY-lasty)/ACtime), 2)
        );

        lasty1 = y1;
        lastx = currentX;
        lasty = currentY;



        ACtimer.reset();


    }

    public boolean isFinished() {
        return ((spline.getEndPoint().getX()-x< translational_error && spline.getEndPoint().getY()-y< translational_error)
                &&(Math.abs(heading-currentHeading)<= heading_error));
    }
}

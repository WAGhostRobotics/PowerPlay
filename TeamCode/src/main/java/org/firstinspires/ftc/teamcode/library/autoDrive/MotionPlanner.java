package org.firstinspires.ftc.teamcode.library.autoDrive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.MergedBezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;


public class MotionPlanner {

    private Bezier spline;
    private double targetHeading;

    private Drivetrain drive;
    private Localizer localizer;

//    private PIDController translationalControl = new PIDController(0.022,0.001,0.03);
    public static PIDController translationalControl = new PIDController(0.21,.05,0.064);
    public static PIDController headingControl = new PIDController(0.01, 0.042, 0.005);

//    private PIDController translationalControlEnd = new PIDController(0.022,0.001,0.03);
    public static PIDController translationalControlEnd = new PIDController(0.12,0.1,0.03);
    public static PIDController headingControlEnd = new PIDController(0.028, 0.12, 0.005);


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
    public final double THE_HOLY_CONSTANT = 0.001; //0.001

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
    private final double translational_error = 1;
    private final double heading_error = 1;

    private final double endTrajThreshhold = 12;

    boolean end = false;

    private ElapsedTime timer;
    private ElapsedTime ACtimer;

    private double distanceLeft;
    private double estimatedStopping;

    Point target;
    Point derivative;

    double perpendicularError;
    private final double tIncrement = 0.05;

    HardwareMap hwMap;

    double voltage = 0;

    public MotionPlanner(MecanumDrive drive, Localizer localizer, HardwareMap hwMap){

        this.drive = drive;
        this.localizer = localizer;
        this.hwMap = hwMap;


    }

    public void reset(){
        timer = new ElapsedTime();
        ACtimer = new ElapsedTime();

//        t1 = MAX_VEL/MAX_ACCEL; // time to accelerate
//        t3 = MAX_VEL/MAX_ACCEL; // time to decelerate
//
//        t2 = (spline.approximateLength() - 2 * (0.5 * MAX_ACCEL * Math.pow(t1, 2)))/MAX_VEL; // time in da middle
//
//
//        if(t2<0){
//            t2 = 0;
//        }
//        time = t1 + t2 + t3;


        translationalControl.reset();
        headingControl.reset();

        voltage = hwMap.voltageSensor.iterator().next().getVoltage();

        numLoops = 0;
        loopTime = new ElapsedTime();

        double length = spline.approximateLength();
        estimatedStopping = (length - endTrajThreshhold)/length;

        t = 0;
    }

    public void startTrajectory(Bezier spline) {
        this.spline = new Bezier(spline);

        reset();
    }

    public void startTrajectory(Bezier... splines) {
        this.spline = new MergedBezier(splines);
        reset();

    }

    public Bezier getSpline(){
        return spline;
    }


    public String getTelemetry(){
        return "T: " + t +
//                "\n Theta: " + theta +
//                "\n Magnitude: " + magnitude +
//                "\n Phase: " + end +
//                "\n Stop " + (distanceLeft < estimatedStopping) +
//                "\n Distance left: " + distanceLeft +
//                "\n Distance left (x): " + (spline.getEndPoint().getX()-x) +
//                "\n Distance left (y): " + (spline.getEndPoint().getY()-y) +
                "\n Perpendicular error: " + (perpendicularError) +
//                "\n Heading: " + (heading - currentHeading) +
                "\n Estimated Stopping " + estimatedStopping +
//                "\n " + drive.getTelemetry() +
                "\n Finished " + isFinished()+
                "\n Loop Rate " + numLoops/loopTime.seconds();
    }

    public double getPerpendicularError(){
        return perpendicularError;
    }


    public double getHeadingError(){
        double headingError = targetHeading - currentHeading;

        if(headingError > 180){
            headingError -= 360;
        }else if(headingError<-180){
            headingError += 360;
        }

        return headingError;
    }

    public void update() {

        localizer.update();
        updateACValues();

        x = localizer.getX();
        y = localizer.getY();
        currentHeading = normalizeDegrees(localizer.getHeading(Localizer.Angle.DEGREES));

//        t = timer.seconds()/time;

        while(t <= estimatedStopping && distance(spline.getPoint(t + tIncrement), new Point(x, y))<
                distance(spline.getPoint(t), new Point(x, y))){
            t += tIncrement;
        }

        target = spline.getPoint(t);
        targetHeading = spline.getHeading(t);
        derivative = spline.getDerivative(t);


        if(!isFinished()){

            distanceLeft = distance(spline.getEndPoint(), new Point(x, y));

            if(distanceLeft <= endTrajThreshhold||t>=estimatedStopping){



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
                driveTurn = headingControlEnd.calculate(0, getHeadingError());



                drive.drive(magnitude, theta, driveTurn, movementPower, voltage);


            } else {

                end = false;

                magnitude = 1;

                double vy = derivative.getY();
                double vx = derivative.getX();

                theta = Math.toDegrees(Math.atan2(vy, vx));


                /********DUMB*********/

                double correction;
                double multiplier = 1;

                if(vx == 0){


                    perpendicularError = target.getX() - x;


                    if(vy<0){
                        multiplier = -1;
                    }


                }else{
                    double slope = vy/vx;
                    double yIntTarget = (target.getY() - (slope)*(target.getX()));
                    double yIntReal = (y - (slope)*x);

                    perpendicularError = Math.abs(yIntTarget-yIntReal)/Math.sqrt(1 + Math.pow(slope, 2));
                    perpendicularError = Math.signum(normalizeDegrees(theta-90)) * perpendicularError;

                    if(yIntTarget <= yIntReal){
                        multiplier = -1;
                    }

                }
                perpendicularError *= multiplier;
                correction = translationalControl.calculate(0, perpendicularError);
                theta -= Math.toDegrees(Math.atan2(correction, magnitude));
                magnitude = Math.hypot(magnitude, correction);


                x_power = magnitude * Math.cos(Math.toRadians(theta));
                y_power = magnitude * Math.sin(Math.toRadians(theta));



                x_rotated = x_power * Math.cos(Math.toRadians(currentHeading)) + y_power * Math.sin(Math.toRadians(currentHeading));
                y_rotated = -x_power * Math.sin(Math.toRadians(currentHeading)) + y_power * Math.cos(Math.toRadians(currentHeading));

                magnitude = Math.hypot(x_rotated, y_rotated);
                theta = Math.toDegrees(Math.atan2(y_rotated, x_rotated));
                driveTurn = headingControl.calculate(0, getHeadingError());


                if(!Double.isNaN(y1)&&!Double.isNaN(y2) && magnitude != 0){
                    radius = Math.pow((1+Math.pow(y1,2)), 1.5)/y2;
                    ac = Math.pow(velocity, 2)/radius;
                    theta -= Math.toDegrees(Math.atan2( ac*THE_HOLY_CONSTANT, 1));
                    magnitude *= Math.hypot(1, ac*THE_HOLY_CONSTANT);

                }else{
                    ac = 0;
                }

                drive.driveMax(magnitude, theta, driveTurn, movementPower, voltage);
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
        return ((Math.abs(spline.getEndPoint().getX()-x)<= translational_error && Math.abs(spline.getEndPoint().getY()-y)<= translational_error)
                &&(Math.abs(targetHeading - currentHeading)<= heading_error));
    }

    private double distance(Point p1, Point p2){
        return Math.hypot(p1.getX()-p2.getX(), p1.getY()-p2.getY());
    }
}

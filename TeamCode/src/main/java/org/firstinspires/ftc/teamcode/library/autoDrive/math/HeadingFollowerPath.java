package org.firstinspires.ftc.teamcode.library.autoDrive.math;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

public class HeadingFollowerPath extends Bezier{

    public static boolean followForward = true;

    public HeadingFollowerPath(boolean followForward, Point... waypoints){
        super(waypoints);
        this.followForward = followForward;
    }

    @Override
    public double getHeading(double t) {
        double heading = normalizeDegrees(Math.toDegrees(Math.atan2(getDerivative(t).getY(), getDerivative(t).getX())));

        if(!followForward){
            heading = normalizeDegrees(heading - 180);
        }

        return heading;
    }
}

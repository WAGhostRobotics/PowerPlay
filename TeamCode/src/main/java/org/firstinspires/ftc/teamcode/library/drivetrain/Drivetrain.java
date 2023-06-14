package org.firstinspires.ftc.teamcode.library.drivetrain;

public interface Drivetrain {
    public void driveMax(double magnitude, double theta, double driveTurn, double movementPower);
    public void drive(double magnitude, double theta, double driveTurn, double movementPower);

    public default void driveMax(double magnitude, double theta, double driveTurn, double movementPower, double voltage) {

    }

    public default void drive(double magnitude, double theta, double driveTurn, double movementPower, double voltage) {

    }

    public String getTelemetry();
}

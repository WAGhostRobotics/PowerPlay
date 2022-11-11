package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Light {
    private DcMotor light;

    public void init(HardwareMap hardwareMap) {
        light = hardwareMap.get(DcMotor.class, "light");
    }

    public void on() {
        light.setPower(0.15);
    }

    public void off() {
        light.setPower(0);
    }
}

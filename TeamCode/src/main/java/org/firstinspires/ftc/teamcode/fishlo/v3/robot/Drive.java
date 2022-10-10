package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Drive extends SubSystem {

    private SampleMecanumDrive mDrive = new SampleMecanumDrive(robot.hardwareMap);


    public Drive(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {

    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {

    }

    public SampleMecanumDrive getInstance() {
        return mDrive;
    }
}

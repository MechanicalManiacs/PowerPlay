package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Drive extends SubSystem {

    private SampleMecanumDrive mDrive;

    public Drive(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        mDrive = new SampleMecanumDrive(robot.hardwareMap);
    }

    @Override
    public void handle() {
        mDrive.setWeightedDrivePower(
                new Pose2d(
                        -robot.gamepad1.left_stick_y,
                        -robot.gamepad1.left_stick_x,
                        -robot.gamepad1.right_stick_x
                )
        );
    }

    @Override
    public void stop() {}

    public SampleMecanumDrive getInstance() {
        return mDrive;
    }
}

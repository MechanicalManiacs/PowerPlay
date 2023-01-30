package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.firstinspires.ftc.teamcode.rr.drive.StandardTrackingWheelLocalizer;

import java.util.Vector;

public class Drive extends SubSystem {

    private SampleMecanumDrive mDrive;
    private ServoEx leftSpool;
    private ServoEx rightSpool;
    private ServoEx frontSpool;
    private DriveType driveType;
    boolean first = true;
    boolean odoLifted = false;

    public Drive(Robot robot) {
        super(robot);
    }

    public enum DriveType {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    @Override
    public void init() {
        leftSpool = robot.hardwareMap.get(ServoEx.class, "leftspool");
        rightSpool = robot.hardwareMap.get(ServoEx.class, "rightspool");
        frontSpool = robot.hardwareMap.get(ServoEx.class, "frontspool");
        mDrive = new SampleMecanumDrive(robot.hardwareMap);
        driveType = DriveType.ROBOT_CENTRIC;
        robot.gamepad1.type = Gamepad.Type.SONY_PS4;
        robot.gamepad2.type = Gamepad.Type.SONY_PS4;
    }

    @Override
    public void handle() {
//        if (first) {
//            leftSpool.rotateByAngle(360);
//            rightSpool.rotateByAngle(-360);
//            frontSpool.rotateByAngle(360);
//            first = false;
//            odoLifted = true;
//        }
        if (robot.gamepad1.touchpad_finger_1) driveType = DriveType.ROBOT_CENTRIC;
        if (robot.gamepad1.touchpad_finger_2 && !odoLifted) driveType = DriveType.FIELD_CENTRIC;
        double leftX = -robot.gamepad1.left_stick_x;
        double leftY = -robot.gamepad1.left_stick_y;
        double rightX = robot.gamepad1.right_stick_x/2;
        Pose2d drivePowers = new Pose2d();
        switch (driveType) {
            case ROBOT_CENTRIC:
                Vector2d translation = new Vector2d(leftX, leftY).rotated(-mDrive.getPoseEstimate().getHeading());
                drivePowers = new Pose2d(translation.getX(), translation.getY(), rightX);
                break;
            case FIELD_CENTRIC:
                Vector2d translation2 = new Vector2d(leftX, leftY);
                drivePowers = new Pose2d(translation2.getX(), translation2.getY(), rightX);
                break;
        }
        mDrive.setWeightedDrivePower(drivePowers);
    }

    @Override
    public void stop() {}

    public SampleMecanumDrive getInstance() {
        return mDrive;
    }
}

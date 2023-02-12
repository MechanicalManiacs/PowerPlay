package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.rr.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.firstinspires.ftc.teamcode.rr.drive.StandardTrackingWheelLocalizer;

import java.util.Vector;

public class Drive extends SubSystem {

    private SampleMecanumDrive mDrive = new SampleMecanumDrive(robot.hardwareMap);
    private ServoEx leftSpool;
    private ServoEx rightSpool;
    private ServoEx frontSpool;
    private DriveType driveType;
    private DriveType prevDriveType;
    boolean first = true;
    boolean odoLifted = false;

    private Vector2d targetPosition;
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    public Drive(Robot robot) {
        super(robot);
    }

    public enum DriveType {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        ALIGN_TO_HIGH_JUNCTION
    }

    @Override
    public void init() {
        Pose2d startPose = new Pose2d(-72, -36, 0);
        leftSpool = robot.hardwareMap.get(ServoEx.class, "leftspool");
        rightSpool = robot.hardwareMap.get(ServoEx.class, "rightspool");
        frontSpool = robot.hardwareMap.get(ServoEx.class, "frontspool");
        driveType = DriveType.ROBOT_CENTRIC;
        prevDriveType = DriveType.ROBOT_CENTRIC;
        robot.gamepad1.type = Gamepad.Type.SONY_PS4;
        robot.gamepad2.type = Gamepad.Type.SONY_PS4;
        mDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        mDrive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        mDrive.getLocalizer().setPoseEstimate(startPose);
        headingController.setInputBounds(-Math.PI, Math.PI);
        targetPosition = PoseStorage.alliance.highJunctionPosition;
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
        Pose2d poseEstimate = mDrive.getPoseEstimate();
        switch (driveType) {
            case FIELD_CENTRIC:
                if (robot.gamepad1.y) driveType = DriveType.ALIGN_TO_HIGH_JUNCTION;
                prevDriveType = DriveType.FIELD_CENTRIC;
                Vector2d translation = new Vector2d(leftX, leftY).rotated(-poseEstimate.getHeading());
                drivePowers = new Pose2d(translation.getX(), translation.getY(), rightX);
                break;
            case ROBOT_CENTRIC:
                if (robot.gamepad1.y) driveType = DriveType.ALIGN_TO_HIGH_JUNCTION;
                prevDriveType = DriveType.ROBOT_CENTRIC;
                Vector2d translation2 = new Vector2d(leftX, leftY);
                drivePowers = new Pose2d(translation2.getX(), translation2.getY(), rightX);
                break;
            case ALIGN_TO_HIGH_JUNCTION:
                if (robot.gamepad1.a) driveType = prevDriveType;
                // Create a vector from the gamepad x/y inputs which is the field relative movement
                // Then, rotate that vector by the inverse of that heading for field centric control
                Vector2d fieldFrameInput = new Vector2d(
                        -robot.gamepad1.left_stick_y,
                        -robot.gamepad1.left_stick_x
                );
                Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                // Difference between the target vector and the bot's position
                Vector2d difference = targetPosition.minus(poseEstimate.vec());
                // Obtain the target angle for feedback and derivative for feedforward
                double theta = difference.angle();

                // Not technically omega because its power. This is the derivative of atan2
                double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                // Set the target heading for the heading controller to our desired angle
                headingController.setTargetPosition(theta);

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV + thetaFF)
                        * DriveConstants.TRACK_WIDTH;

                // Combine the field centric x/y velocity with our derived angular velocity
                drivePowers = new Pose2d(
                        robotFrameInput,
                        headingInput
                );
        }
        mDrive.setWeightedDrivePower(drivePowers);
        mDrive.getLocalizer().update();
    }

    @Override
    public void stop() {}

    public SampleMecanumDrive getInstance() {
        return mDrive;
    }
}

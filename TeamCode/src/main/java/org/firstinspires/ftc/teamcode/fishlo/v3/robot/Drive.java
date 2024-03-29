package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.rr.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.firstinspires.ftc.teamcode.rr.drive.StandardTrackingWheelLocalizer;

import java.util.Vector;

public class Drive extends SubSystem {

    private SampleMecanumDrive mDrive;
    private DriveType driveType;

    private Vector2d targetPosition;
    private PIDFController headingController;

    private Vector2d farHighJunction;
    private Vector2d nearHighJunction;
    private Vector2d leftHighJunction;
    private Vector2d rightHighJunction;

    private DistanceSensor distanceSensor;

    private LED ledLight1;
    private LED ledlight2;

    private boolean first = false;

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
        mDrive = new SampleMecanumDrive(robot.hardwareMap);
        driveType = DriveType.ROBOT_CENTRIC;
        mDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mDrive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
        farHighJunction = new Vector2d(-24, 0);
        nearHighJunction = new Vector2d(24, 0);
        leftHighJunction = new Vector2d(0, -24);
        rightHighJunction = new Vector2d(0, 24);
        headingController.setInputBounds(-Math.PI, Math.PI);
        distanceSensor = robot.hardwareMap.get(DistanceSensor.class, "distance");
        ledLight1 = robot.hardwareMap.led.get("led1");
        ledlight2 = robot.hardwareMap.led.get("led2");
    }

    @Override
    public void handle() {
        double leftY = applyDeadzone(-robot.gamepad1.left_stick_x*0.75);
        double leftX = applyDeadzone(-robot.gamepad1.left_stick_y*0.75);
        double rightX = applyDeadzone(-robot.gamepad1.right_stick_x/2);
        Pose2d drivePowers = new Pose2d();
        if (robot.gamepad1.y) driveType = DriveType.ALIGN_TO_HIGH_JUNCTION;
        if (robot.gamepad1.b) driveType = DriveType.FIELD_CENTRIC;
        Vector2d translation2 = new Vector2d(leftX, leftY);
        drivePowers = new Pose2d(translation2.getX(), translation2.getY(), rightX);
        mDrive.setWeightedDrivePower(drivePowers);
    }

    @Override
    public void stop() {}

    public SampleMecanumDrive getInstance() {
        return mDrive;
    }

    public void strafeUntilAligned(String direction) {
        if (direction.equalsIgnoreCase("LEFT")) {
            while (distanceSensor.getDistance(DistanceUnit.INCH) > 10) {
                robot.telemetry.addData("DISTANCE SENSOR", distanceSensor.getDistance(DistanceUnit.INCH));
                robot.telemetry.update();
                mDrive.setWeightedDrivePower(new Pose2d(0, 0.2, 0));
            }
        } else if (direction.equalsIgnoreCase("RIGHT")) {
            while (distanceSensor.getDistance(DistanceUnit.INCH) > 10) {
                robot.telemetry.addData("DISTANCE SENSOR", distanceSensor.getDistance(DistanceUnit.INCH));
                robot.telemetry.update();
                mDrive.setWeightedDrivePower(new Pose2d(0, -0.2, 0));
            }
        }
    }

    public void strafeUntilAligned(String direction, double speed) {
        if (direction.equalsIgnoreCase("LEFT")) {
            while (distanceSensor.getDistance(DistanceUnit.INCH) > 10) {
                robot.telemetry.addData("DISTANCE SENSOR", distanceSensor.getDistance(DistanceUnit.INCH));
                robot.telemetry.update();
                mDrive.setWeightedDrivePower(new Pose2d(0, speed, 0));
            }
        } else if (direction.equalsIgnoreCase("RIGHT")) {
            while (distanceSensor.getDistance(DistanceUnit.INCH) > 10) {
                robot.telemetry.addData("DISTANCE SENSOR", distanceSensor.getDistance(DistanceUnit.INCH));
                robot.telemetry.update();
                mDrive.setWeightedDrivePower(new Pose2d(0, -speed, 0));
            }
        }
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public double applyDeadzone(double value) {
        double deadzone = 0.02;
        return Math.abs(value) > deadzone ? value : 0;
    }

    public void lightForDistance() {
        if (getDistance() > 1 && getDistance() <= 5.5) {
            ledLight1.enableLight(true);
            ledlight2.enableLight(true);
        }
        else {
            ledLight1.enableLight(false);
            ledlight2.enableLight(false);
        }
    }
}

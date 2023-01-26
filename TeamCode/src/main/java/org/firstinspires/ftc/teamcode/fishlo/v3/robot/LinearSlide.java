package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class LinearSlide extends SubSystem {

    DcMotorEx lift;
    Servo claw;
 //   Servo arm;

    private double ticksPerRev = 384.5;

    public enum Level {
        LOW(750.0),
        MID(1750.0),
        HIGH(2000.0),
        GROUND(50.0),
        RESET(0.0);

        double ticks;
        private Level(double ticks) {
            this.ticks = ticks;
        }
    }

    public enum ClawPos {
        OPEN(0.4),
        CLOSED(0.0);

        final double pos;
        private ClawPos(double pos) {
            this.pos = pos;
        }
    }

    /**
     * Construct a subsystem with the robot it applies to.
     *
     * @param robot
     */
    public LinearSlide(Robot robot) {
        super(robot);
    }

    @Override

    public void init() {
        lift = robot.hardwareMap.get(DcMotorEx.class, "lift");
        claw = robot.hardwareMap.servo.get("claw");
        
   //     arm = robot.hardwareMap.servo.get("arm");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setTargetPositionTolerance(2);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setClaw(ClawPos.CLOSED);
    }

    @Override
    public void handle() {
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        joystickMoveWithLimits(-robot.gamepad2.left_stick_y*0.7);
//        lift.setPower(-robot.gamepad2.left_stick_y*0.7);
//        robot.telemetry.addData("ENCODER OF LIFT", lift.getCurrentPosition());
//        robot.telemetry.update();
        if (robot.gamepad2.a) setClaw(ClawPos.OPEN);
        if (robot.gamepad2.b) setClaw(ClawPos.CLOSED);
    }

    @Override
    public void stop() {
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveAndDrop(Level level) {
//        lift.setPower(targetPower);
    }

    public void joystickMoveWithLimits(double leftStickY) {
        robot.telemetry.addData("ENCODER OF LIFT", lift.getCurrentPosition());
        robot.telemetry.update();
        if (leftStickY <= 0 && lift.getCurrentPosition() <= 5) {
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else {
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(leftStickY);
        }
    }

//    public void turnArm(double degrees) {
//        if (degrees < 0) {
//            arm.setDirection(Servo.Direction.REVERSE);
//            degrees = -degrees;
//        }
//        else arm.setDirection(Servo.Direction.FORWARD);
//        double clamped = Range.scale(degrees, 0, 180, 0, 1);
//        arm.setPosition(clamped);
//    }

    public void resetEncoder() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setClaw(ClawPos pos) {
        claw.setPosition(pos.pos);
    }

    private int rotationsToTicks(double rotations) {
        return (int) (ticksPerRev * rotations);
    }
}

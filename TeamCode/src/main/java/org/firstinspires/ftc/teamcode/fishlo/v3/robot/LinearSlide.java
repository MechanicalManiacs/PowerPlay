package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class LinearSlide extends SubSystem {

    DcMotorEx lift;
    Servo claw;
 //   Servo arm;

    private double ticksPerRev = 384.5;

    public enum Level {
        LOW(-750.0),
        MID(-1750.0),
        HIGH(-2000.0),
        GROUND(-50.0),
        RESET(0.0),
        CUSTOM(0.0);

        double rotations;
        private Level(double rotations) {
            this.rotations = rotations;
        }
        public Level setCustom(double val) {
            CUSTOM.rotations = val;
            return CUSTOM;
        }
    }

    public enum ClawPos {
        OPEN(0.0),
        CLOSED(1.0);

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
        lift = robot.hardwareMap.get(DcMotorEx.class, "lift1");
        claw = robot.hardwareMap.servo.get("claw");
   //     arm = robot.hardwareMap.servo.get("arm");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setTargetPositionTolerance(2);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setClaw(ClawPos.OPEN);
    }

    @Override
    public void handle() {
        joystickMoveWithLimits(-robot.gamepad2.left_stick_y);
        robot.telemetry.addData("ENCODER_1", lift.getCurrentPosition());
        robot.telemetry.update();
        if (robot.gamepad2.a) setClaw(ClawPos.OPEN);
        if (robot.gamepad2.b) setClaw(ClawPos.CLOSED);
  //      arm.setPosition(robot.gamepad2.right_stick_x);
    }

    @Override
    public void stop() {
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void moveAndDrop(Level level) {
        switch (level) {
            case LOW:
                lift.setTargetPosition(rotationsToTicks(Level.LOW.rotations));
                break;
            case MID:
                lift.setTargetPosition(rotationsToTicks(Level.MID.rotations));
                break;
            case HIGH:
                lift.setTargetPosition(rotationsToTicks(Level.HIGH.rotations));
                break;
            case RESET:
                lift.setTargetPosition(rotationsToTicks(Level.RESET.rotations));
                break;
        }
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.setPower(1);
    }

    public void joystickMoveWithLimits(double leftStickY) {
        if (leftStickY > 0 && lift.getCurrentPosition() < rotationsToTicks(Level.HIGH.rotations)) {
            lift.setPower(leftStickY);
        } else if (leftStickY < 0 && lift.getCurrentPosition() > rotationsToTicks(Level.RESET.rotations)) {
            lift.setPower(leftStickY);
        } else {
            lift.setPower(0);
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

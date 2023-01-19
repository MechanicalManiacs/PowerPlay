package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class LinearSlide extends SubSystem {

    DcMotorEx lift1;
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
        lift1 = robot.hardwareMap.get(DcMotorEx.class, "lift1");
        claw = robot.hardwareMap.servo.get("claw");
   //     arm = robot.hardwareMap.servo.get("arm");
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setTargetPositionTolerance(2);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setClaw(ClawPos.OPEN);
    }

    @Override
    public void handle() {
        lift1.setPower(-robot.gamepad2.right_stick_y);


        robot.telemetry.addData("ENCODER_1", lift1.getCurrentPosition());
        robot.telemetry.update();
        if (robot.gamepad2.a) setClaw(ClawPos.OPEN);
        if (robot.gamepad2.b) setClaw(ClawPos.CLOSED);
  //      arm.setPosition(robot.gamepad2.right_stick_x);
    }

    @Override
    public void stop() {
        lift1.setPower(0);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void moveAndDrop(Level level) {
        switch (level) {
            case LOW:
                lift1.setTargetPosition(rotationsToTicks(Level.LOW.rotations));
                break;
            case MID:
                lift1.setTargetPosition(rotationsToTicks(Level.MID.rotations));
                break;
            case HIGH:
                lift1.setTargetPosition(rotationsToTicks(Level.HIGH.rotations));
                break;
            case RESET:
                lift1.setTargetPosition(rotationsToTicks(Level.RESET.rotations));
                break;
        }
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setPower(1);
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

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setClaw(ClawPos pos) {
        claw.setPosition(pos.pos);
    }

    private int rotationsToTicks(double rotations) {
        return (int) (ticksPerRev * rotations);
    }
}

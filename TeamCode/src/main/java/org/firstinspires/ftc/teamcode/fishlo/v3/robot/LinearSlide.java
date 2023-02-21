package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.util.IntegerSequence;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils.MotionConstraint;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.opencv.core.Range;

@Config
public class LinearSlide extends SubSystem {

    DcMotorEx lift;
    Servo claw;

    private static PIDCoefficients positionCoefficients = new PIDCoefficients(0, 0, 0);
    private static MotionConstraint upConstraint = new MotionConstraint(0, 0, 0);
    private static MotionConstraint downConstraint = new MotionConstraint(0, 0, 0);

    private static PIDCoefficients velCoefficients = new PIDCoefficients(0, 0, 0);

    private boolean first = true;

    double slideTargetPosition = 0;

    private double ticksPerRev = 384.5;

    private final double SLIDE_MAX = 3100;
    private final double SLIDE_MIN = 20;

    private Level currentLevel;

    private boolean override = false;

    public enum Level {
        LOW(750),
        MID(1750),
        HIGH(3050),
        GROUND(50),
        RESET(0),
        CYCLE_POS_1(460),
        CYCLE_POS_2(350),
        CYCLE_POS_3(250),
        CYCLE_POS_4(100),
        CYCLE_POS_5(0);

        int ticks;
        private Level(int ticks) {
            this.ticks = ticks;
        }

        public Level next() {
            Level returnLevel = RESET;
            switch (this) {
                case CYCLE_POS_1:
                    returnLevel = CYCLE_POS_2;
                    break;
                case CYCLE_POS_2:
                    returnLevel = CYCLE_POS_3;
                    break;
                case CYCLE_POS_3:
                    returnLevel = CYCLE_POS_4;
                    break;
                case CYCLE_POS_4:
                    returnLevel = CYCLE_POS_5;
                    break;
                case CYCLE_POS_5:
                    returnLevel = CYCLE_POS_1;
                    break;
                case RESET:
                    returnLevel = CYCLE_POS_1;
                    break;

            }
            return returnLevel;
        }

        public Level prev() {
            Level returnLevel = RESET;
            switch (this) {
                case CYCLE_POS_1:
                    returnLevel = CYCLE_POS_5;
                    break;
                case CYCLE_POS_2:
                    returnLevel = CYCLE_POS_1;
                    break;
                case CYCLE_POS_3:
                    returnLevel = CYCLE_POS_2;
                    break;
                case CYCLE_POS_4:
                    returnLevel = CYCLE_POS_3;
                    break;
                case CYCLE_POS_5:
                    returnLevel = CYCLE_POS_4;
                    break;
                case RESET:
                    returnLevel = CYCLE_POS_1;
                    break;
            }
            return returnLevel;
        }
    }

    public enum ClawPos {
        OPEN(1.0),
        CLOSED(0.0);

        double pos;
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
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setClaw(ClawPos.CLOSED);
        currentLevel = Level.RESET;
    }

    @Override
    public void handle() {
        if (robot.gamepad2.right_bumper) {
            override = true;
        }
        if (robot.gamepad2.left_bumper) {
            override = false;
        }
        if (override) {
            lift.setPower(-robot.gamepad2.left_stick_y);
        }
        else if (!override) {
            moveJoyLimits(-robot.gamepad2.left_stick_y);
        }
        if (robot.gamepad2.x) setClaw(ClawPos.CLOSED);
        if (robot.gamepad2.b) {
            setClaw(ClawPos.OPEN);
        }
    }

    public void moveJoyLimits(double leftStickY) {
//        robot.telemetry.addData("LIFT ENCODER", lift.getCurrentPosition());
//        robot.telemetry.update();
        if (leftStickY < 0 && lift.getCurrentPosition() <= SLIDE_MIN) {
            lift.setPower(0);
        }
        else if (leftStickY > 0 && lift.getCurrentPosition() >= SLIDE_MAX) {
            lift.setPower(0.1);
        }
        else {
            lift.setPower(leftStickY);
        }
    }

    @Override
    public void stop() {
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void liftSlide(Level level, double speed) {
        lift.setTargetPosition(level.ticks);
        lift.setPower(speed);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveVelocityJoy(double leftStickY) {
        lift.setVelocity(leftStickY*2800);
    }

    public void resetEncoder() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setClaw(ClawPos pos) {
        claw.setPosition(pos.pos);
    }
}

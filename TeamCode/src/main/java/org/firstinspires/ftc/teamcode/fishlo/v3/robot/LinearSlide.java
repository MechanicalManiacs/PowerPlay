package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils.MotionConstraint;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils.ProfiledPID;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

@Config
public class LinearSlide extends SubSystem {

    DcMotorEx lift;
    Servo claw;

    private static PIDCoefficients positionCoefficients = new PIDCoefficients(0, 0, 0);
    private static MotionConstraint upConstraint = new MotionConstraint(0, 0, 0);
    private static MotionConstraint downConstraint = new MotionConstraint(0, 0, 0);

    private static PIDCoefficients velCoefficients = new PIDCoefficients(0, 0, 0);

    private boolean first = true;

    ProfiledPID controller = new ProfiledPID(upConstraint, downConstraint, positionCoefficients);

    double slideTargetPosition = 0;

    private double ticksPerRev = 384.5;

    public enum Level {
        LOW(750.0),
        MID(1750.0),
        HIGH(2700.0),
        GROUND(50.0),
        RESET(0.0),
        CYCLE_POS_1(0.0),
        CYCLE_POS_2(0.0),
        CYCLE_POS_3(0.0),
        CYCLE_POS_4(0.0),
        CYCLE_POS_5(0.0),
        END(0.0);

        double ticks;
        private Level(double ticks) {
            this.ticks = ticks;
        }

        public Level next() {
            switch (this) {
                case CYCLE_POS_5:
                    return CYCLE_POS_4;
                case CYCLE_POS_4:
                    return CYCLE_POS_3;
                case CYCLE_POS_3:
                    return CYCLE_POS_2;
                case CYCLE_POS_2:
                    return CYCLE_POS_1;
                case CYCLE_POS_1:
                    return END;
                default:
                    return CYCLE_POS_5;
            }
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
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setVelocityPIDCoefficients(velCoefficients);
        setClaw(ClawPos.OPEN);
    }

    @Override
    public void handle() {
        if (!first) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            first = false;
        }
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
//        moveVelocityJoy(-robot.gamepad2.left_stick_y);
//        lift.setPower(-robot.gamepad2.left_stick_y);
        moveJoyLimits(-robot.gamepad2.left_stick_y);
        if (robot.gamepad2.x) setClaw(ClawPos.OPEN);
        if (robot.gamepad2.b) {
            setClaw(ClawPos.CLOSED);
        }
    }

    public void moveJoyLimits(double leftStickY) {
        robot.telemetry.addData("ENCODER OF LIFT", lift.getCurrentPosition());
        robot.telemetry.addData("LEFT STICK Y", leftStickY);
        robot.telemetry.update();
        if (leftStickY < 0 && lift.getCurrentPosition() <= 20) {
            lift.setPower(0);
        }
        else if (leftStickY >= 0 && lift.getCurrentPosition() >= 3100) {
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

    public void liftSlide(Level level) {
        updateTargetPosition(level.ticks);
    }

    public void moveVelocityJoy(double leftStickY) {
        lift.setVelocity(leftStickY*2800);
    }

    public void setVelocityPIDCoefficients(PIDCoefficients coefficients) {
        lift.setVelocityPIDFCoefficients(coefficients.Kp, coefficients.Ki, coefficients.Kd, 0);
    }

    public void resetEncoder() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setClaw(ClawPos pos) {
        claw.setPosition(pos.pos);
    }

    private int rotationsToTicks(double rotations) {
        return (int) (ticksPerRev * rotations);
    }

    public void update() {
        double measuredPosition = getSlidePosition();
        double power = controller.calculate(slideTargetPosition, measuredPosition);
        lift.setPower(power);
        robot.telemetry.addLine("LIFT")
                .addData("Measured Slide Position", measuredPosition)
                .addData("Target Slide Position", slideTargetPosition)
                .addData("Slide Power", power);
        robot.telemetry.update();
    }

    public double getSlidePosition() {
        return lift.getCurrentPosition();
    }

    public double getSlideTargetPosition() {
        return slideTargetPosition;
    }

    public double getVelocity() {
        return lift.getVelocity();
    }

    public void updateTargetPosition(double targetpos) {
        this.slideTargetPosition = targetpos;
    }

    public boolean isMovementFinished() {
        return controller.isDone();
    }

    public ProfiledPID getController() {
        return controller;
    }

    public void setPower(double power) {
        lift.setPower(power);
    }
}

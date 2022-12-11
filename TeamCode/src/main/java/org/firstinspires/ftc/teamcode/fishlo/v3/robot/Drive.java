package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Drive extends SubSystem {

    private SampleMecanumDrive mDrive;
    int coeff = 1;

    DcMotor frontLeft = robot.hardwareMap.dcMotor.get("frontLeft");
    DcMotor frontRight = robot.hardwareMap.dcMotor.get("frontRight");
    DcMotor backLeft = robot.hardwareMap.dcMotor.get("backLeft");
    DcMotor backRight = robot.hardwareMap.dcMotor.get("backRight");

    public Drive(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        mDrive = new SampleMecanumDrive(robot.hardwareMap);
        frontLeft = robot.hardwareMap.dcMotor.get("frontLeft");
        frontRight = robot.hardwareMap.dcMotor.get("frontRight");
        backLeft = robot.hardwareMap.dcMotor.get("backLeft");
        backRight = robot.hardwareMap.dcMotor.get("backRight");
    }

    boolean first = true;

    @Override
    public void handle() {

        if (first) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            first = false;
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        double fl = 0, bl = 0, fr = 0, br = 0;

        robot.telemetry.addData("Current Mode: ", "ARCADE");
        robot.telemetry.update();
        double y = robot.gamepad1.left_stick_y;
        double x = -robot.gamepad1.left_stick_x;
        double rx = -robot.gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        if (robot.gamepad1.left_trigger >= 0.5) coeff = 2;
        else coeff = 1;

        fl = frontLeftPower / coeff;
        bl = backLeftPower / coeff;
        fr = frontRightPower / coeff;
        br = backRightPower / coeff;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    @Override
    public void stop() {

    }

    public DcMotor[] getMotors() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        return new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
    }

    public SampleMecanumDrive getInstance() {
        return mDrive;
    }
}

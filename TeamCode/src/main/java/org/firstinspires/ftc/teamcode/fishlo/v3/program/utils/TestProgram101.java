package org.firstinspires.ftc.teamcode.fishlo.v3.program.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestProgram101 extends LinearOpMode {

    DcMotorEx lift1, lift2, FL, FR, BL, BR;
    Servo claw, arm;


    @Override
    public void runOpMode() throws InterruptedException {
        lift1 = hardwareMap.get(DcMotorEx.class, "lift");
        FL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        FR = hardwareMap.get(DcMotorEx.class, "frontRight");
        BL = hardwareMap.get(DcMotorEx.class, "backLeft");
        BR = hardwareMap.get(DcMotorEx.class, "backRight");
        claw = hardwareMap.servo.get("claw");
        arm = hardwareMap.servo.get("arm");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) FR.setPower(1);
            if (gamepad1.b) BR.setPower(1);
            if (gamepad1.y) FL.setPower(1);
            if (gamepad1.x) BL.setPower(1);
            if (gamepad1.dpad_up) lift1.setPower(-1);
            if (gamepad1.dpad_down) lift1.setPower(1);
            if (gamepad1.dpad_left) arm.setPosition(0);
            if (gamepad1.dpad_right) arm.setPosition(1);
            if (gamepad1.left_bumper) claw.setPosition(0);
            if (gamepad1.right_bumper) claw.setPosition(1);
        }
    }




}


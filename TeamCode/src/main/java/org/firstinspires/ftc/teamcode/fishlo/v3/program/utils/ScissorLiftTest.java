package org.firstinspires.ftc.teamcode.fishlo.v3.program.utils;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.ScissorLift;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.concurrent.TimeUnit;

@Autonomous
public class ScissorLiftTest extends FishloAutonomousProgram {
    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    DcMotorEx liftInstance;
    ElapsedTime timer;

    @Override
    public void preMain() {
        telemetry.addLine("START");
        telemetry.update();
        liftInstance = lift.getMotorInstance();
        liftInstance.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftInstance.setTargetPosition(-2000);
        liftInstance.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftInstance.setPower(0.5);
        timer = new ElapsedTime();
    }

    @Override
    public void main() {
        timer.reset();
        while (opModeIsActive()) {
            telemetry.addData("TIME", timer.time());
        }
//        liftInstance.setVelocity(15.5*Math.PI, AngleUnit.RADIANS);
//        while(liftInstance.isBusy()) {
//            telemetry.addData("POWER", liftInstance.getPower())
//                            .addData("ENCODER", liftInstance.getCurrentPosition());
//            telemetry.update();
//            telemetry.clear();
//        }
//        telemetry.addLine("DONE");
//        liftInstance.setPower(0);
    }
}

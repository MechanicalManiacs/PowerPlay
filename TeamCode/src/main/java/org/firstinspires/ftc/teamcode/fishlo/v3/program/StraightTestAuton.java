package org.firstinspires.ftc.teamcode.fishlo.v3.program;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class StraightTestAuton extends FishloAutonomousProgram {
    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.addLine("Started");
        telemetry.update();
    }

    @Override
    public void main() {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.reset();
        while (timer.time() < 1.5) {
            for (DcMotor m : drive.getMotors()) {
                m.setPower(-0.75);
            }
        }
        for (DcMotor m : drive.getMotors()) {
            m.setPower(0);
        }
    }
}

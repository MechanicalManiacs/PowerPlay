package org.firstinspires.ftc.teamcode.fishlo.v3.program;

import org.firstinspires.ftc.teamcode.fishlo.v3.robot.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class VisionTest extends FishloAutonomousProgram {

    VisionPipeline.ConePosition pos = VisionPipeline.ConePosition.NULL;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.addLine("Initialzed");
        telemetry.update();
        telemetry.setAutoClear(true);
        vision.initVision();
        while(!isStarted()) {
            pos = vision.getConePosition();
            telemetry.addData("Cone Position", pos);
            telemetry.update();
        }
    }

    @Override
    public void main() {
        telemetry.addData("Last Recieved Position: ", pos);
        telemetry.update();
    }
}

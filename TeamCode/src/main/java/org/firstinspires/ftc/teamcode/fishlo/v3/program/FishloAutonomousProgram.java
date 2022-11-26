package org.firstinspires.ftc.teamcode.fishlo.v3.program;

import org.firstinspires.ftc.teamcode.fishlo.v3.robot.Drive;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.Fishlo;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.Vision;
import org.firstinspires.ftc.teamcode.opMode.AutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class FishloAutonomousProgram extends AutonomousProgram {

    protected Vision vision;
    protected Drive drive;

    @Override
    protected Robot buildRobot() {
        Fishlo fishlo = new Fishlo(this);
        vision = (Vision) fishlo.getSubSystem("Vision");
        drive = (Drive) fishlo.getSubSystem("Drive");
        return fishlo;
    }

    @Override
    public void preMain() {}

    @Override
    public void main() {}
}

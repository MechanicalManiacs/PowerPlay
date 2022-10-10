package org.firstinspires.ftc.teamcode.fishlo.v3.program;

import org.firstinspires.ftc.teamcode.fishlo.v3.robot.Fishlo;
import org.firstinspires.ftc.teamcode.opMode.AutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class FishloAutonomousProgram extends AutonomousProgram {

    @Override
    protected Robot buildRobot() {
        Fishlo fishlo = new Fishlo(this);

        return fishlo;
    }

    @Override
    public void main() {}

    @Override
    public void preMain() {}
}

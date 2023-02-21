package org.firstinspires.ftc.teamcode.fishlo.v3.program.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fishlo.v3.robot.Fishlo;
import org.firstinspires.ftc.teamcode.opMode.DriverControlledProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

import java.util.HashMap;

@TeleOp
public class Record extends DriverControlledProgram {

    private Fishlo fishlo;

    @Override
    protected Robot buildRobot() {
        fishlo = new Fishlo(this);
        return fishlo;
    }

}

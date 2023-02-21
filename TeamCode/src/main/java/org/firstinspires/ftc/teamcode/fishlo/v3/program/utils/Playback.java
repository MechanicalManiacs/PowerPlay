package org.firstinspires.ftc.teamcode.fishlo.v3.program.utils;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.io.IOUtils;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.Fishlo;
import org.firstinspires.ftc.teamcode.opMode.DriverControlledProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.InputStream;
import java.util.Scanner;

@TeleOp
public class Playback extends DriverControlledProgram {

    Fishlo fishlo;

    @Override
    protected Robot buildRobot() {
        fishlo = new Fishlo(this);
        Robot.isPlaying = true;
        return fishlo;
    }
}

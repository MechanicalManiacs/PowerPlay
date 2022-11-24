package org.firstinspires.ftc.teamcode.fishlo.v3.program.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.dfp.DfpField;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

@Autonomous
@Config
public class VisionTest extends FishloAutonomousProgram {

    VisionPipeline.ConePosition pos;

    double sum;
    double count;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.addLine("Initialized");
        telemetry.update();
        telemetry.setAutoClear(true);
//        sum = 0;
//        count = 0;
        while (!isStarted()) {
            pos = vision.getConePosition();
            telemetry.addData("Cone Position", pos);
            telemetry.update();
            System.out.println("Init pos: " + pos);
//            String p = pos.toString();
//            int len = p.length();
//            int num = Integer.parseInt(Character.toString(p.charAt(len-1)));
//            sum += num;
//            count++;
        }
    }

    @Override
    public void main() {
//        double fp = sum/count;
//        String dVal = Double.toString(fp);
//        BigDecimal fp2 = new BigDecimal(dVal);
//        fp2 = fp2.setScale(0, BigDecimal.ROUND_HALF_UP);
//        int finalPos = Integer.parseInt(fp2.toString());
        telemetry.addData("Last Recieved Position", pos);
        telemetry.update();
//        telemetry.addData("Rolling Average Position, ", finalPos);
//        telemetry.update();
        System.out.println("Final: " + pos);
    }


}

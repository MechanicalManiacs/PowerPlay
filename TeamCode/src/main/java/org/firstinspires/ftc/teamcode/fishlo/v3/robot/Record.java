package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import android.os.Environment;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.io.FileWriter;

public class Record extends SubSystem {
    JSONArray joystickData;
    ElapsedTime timer = new ElapsedTime();
    FileWriter writer;
    boolean first = true;

    /**
     * Construct a subsystem with the robot it applies to.
     *
     * @param robot
     */
    public Record(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        joystickData = new JSONArray();
        timer.reset();
        try {
            File file = new File("sdcard/FIRST/record.csv");
            robot.telemetry.addData("PATH", "sdcard/FIRST/record.csv");
            robot.telemetry.update();
            writer = new FileWriter(file, true);
            writer.write("leftY,leftX,rightX,rightY,leftTrigger,rightTrigger,leftBumper,rightBumper,a,b,x,y,dpadUp,dpadDown,dpadLeft,dpadRight,leftStickButton,rightStickButton"+System.lineSeparator());
        } catch (Exception e) {
            robot.telemetry.addLine(e.getStackTrace().toString());
        }
    }

    @Override
    public void handle() {
        robot.telemetry.addLine("Recording");
        robot.telemetry.update();
        JSONObject data = new JSONObject();
        if (first) {
            timer.reset();
            first = false;
        }
        double leftX = robot.gamepad1.left_stick_y;
        double leftY = robot.gamepad1.left_stick_x;
        double rightX = robot.gamepad1.right_stick_x;
        double rightY = robot.gamepad1.right_stick_y;
        double leftTrigger = robot.gamepad1.left_trigger;
        double rightTrigger = robot.gamepad1.right_trigger;
        double leftBumper = robot.gamepad1.left_bumper ? 1 : 0;
        double rightBumper = robot.gamepad1.right_bumper ? 1 : 0;
        double a = robot.gamepad1.a ? 1 : 0;
        double b = robot.gamepad1.b ? 1 : 0;
        double x = robot.gamepad1.x ? 1 : 0;
        double y = robot.gamepad1.y ? 1 : 0;
        double dpadUp = robot.gamepad1.dpad_up ? 1 : 0;
        double dpadDown = robot.gamepad1.dpad_down ? 1 : 0;
        double dpadLeft = robot.gamepad1.dpad_left ? 1 : 0;
        double dpadRight = robot.gamepad1.dpad_right ? 1 : 0;
        double leftStickButton = robot.gamepad1.left_stick_button ? 1 : 0;
        double rightStickButton = robot.gamepad1.right_stick_button ? 1 : 0;

//        double leftY2 = robot.gamepad2.left_stick_y;
//        double leftX2 = robot.gamepad2.left_stick_x;
//        double rightX2 = robot.gamepad2.right_stick_x;
//        double rightY2 = robot.gamepad2.right_stick_y;
//        double leftTrigger2 = robot.gamepad2.left_trigger;
//        double rightTrigger2 = robot.gamepad2.right_trigger;
//        double leftBumper2 = robot.gamepad2.left_bumper ? 1 : 0;
//        double rightBumper2 = robot.gamepad2.right_bumper ? 1 : 0;
//        double a2 = robot.gamepad2.a ? 1 : 0;
//        double b2 = robot.gamepad2.b ? 1 : 0;
//        double x2 = robot.gamepad2.x ? 1 : 0;
//        double y2 = robot.gamepad2.y ? 1 : 0;
//        double dpadUp2 = robot.gamepad2.dpad_up ? 1 : 0;
//        double dpadDown2 = robot.gamepad2.dpad_down ? 1 : 0;
//        double dpadLeft2 = robot.gamepad2.dpad_left ? 1 : 0;
//        double dpadRight2 = robot.gamepad2.dpad_right ? 1 : 0;
//        double leftStickButton2 = robot.gamepad2.left_stick_button ? 1 : 0;
//        double rightStickButton2 = robot.gamepad2.right_stick_button ? 1 : 0;

        double timestamp = timer.milliseconds();

        try {
//            data.put("timestamp", timestamp);
//            data.put("gamepad1", new JSONObject()
//                    .put("leftY", leftY)
//                    .put("leftX", leftX)
//                    .put("rightX", rightX)
//                    .put("rightY", rightY)
//                    .put("leftTrigger", leftTrigger)
//                    .put("rightTrigger", rightTrigger)
//                    .put("leftBumper", leftBumper)
//                    .put("rightBumper", rightBumper)
//                    .put("a", a)
//                    .put("b", b)
//                    .put("x", x)
//                    .put("y", y)
//                    .put("dpadUp", dpadUp)
//                    .put("dpadDown", dpadDown)
//                    .put("dpadLeft", dpadLeft)
//                    .put("dpadRight", dpadRight)
//                    .put("leftStickButton", leftStickButton)
//                    .put("rightStickButton", rightStickButton)
//            );
//            data.put("gamepad2", new JSONObject()
//                    .put("leftY", leftY2)
//                    .put("leftX", leftX2)
//                    .put("rightX", rightX2)
//                    .put("rightY", rightY2)
//                    .put("leftTrigger", leftTrigger2)
//                    .put("rightTrigger", rightTrigger2)
//                    .put("leftBumper", leftBumper2)
//                    .put("rightBumper", rightBumper2)
//                    .put("a", a2)
//                    .put("b", b2)
//                    .put("x", x2)
//                    .put("y", y2)
//                    .put("dpadUp", dpadUp2)
//                    .put("dpadDown", dpadDown2)
//                    .put("dpadLeft", dpadLeft2)
//                    .put("dpadRight", dpadRight2)
//                    .put("leftStickButton", leftStickButton2)
//                    .put("rightStickButton", rightStickButton2)
//            );
            writer.append("" + timestamp)
                    .append("," + leftX)
                    .append("" + leftY)
                    .append("," + leftY)
                    .append("," + rightX)
                    .append("," + rightY)
                    .append("," + leftTrigger)
                    .append("," + rightTrigger)
                    .append("," + leftBumper)
                    .append("," + rightBumper)
                    .append("," + a)
                    .append("," + b)
                    .append("," + x)
                    .append("," + y)
                    .append("," + dpadUp)
                    .append("," + dpadDown)
                    .append("," + dpadLeft)
                    .append("," + dpadRight)
                    .append("," + leftStickButton)
                    .append("," + rightStickButton + System.lineSeparator());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void stop() {
        try {
            writer.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }

//    public void write() {
//        try {
//            writer.write(joystickData.toString());
//            writer.close();
//        } catch (Exception e) {
//            e.printStackTrace();
//        }
//    }
}

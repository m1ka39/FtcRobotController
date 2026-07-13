package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MainHardwareMap.*;

@Autonomous(name="TimedAuto", group="Review")
public class TimedAuto extends LinearOpMode {
    @Override
    public void runOpMode() {

        // Telemetry to show the robot status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables based on your robot configuration
        MainHardwareMap.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // -------------------------
        // AUTONOMOUS ACTIONS BEGIN
        // -------------------------



        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000); // Wait a second before ending
    }
}



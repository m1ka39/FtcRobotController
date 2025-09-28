package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Main Auto", group = "Autonomous")
public class MainAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // === Initialize hardware ===
        MainHardwareMap.init(hardwareMap);

        telemetry.addLine("Hardware Initialized");
        telemetry.update();

        // === Wait for start button ===
        waitForStart();

        //Webcam Usage to get color pattern

        // Drive forward for 1 second
        MainHardwareMap.FrontLeftDTM.setPower(0.5);
        MainHardwareMap.FrontRightDTM.setPower(0.5);
        MainHardwareMap.RearLeftDTM.setPower(0.5);
        MainHardwareMap.RearRightDTM.setPower(0.5);

        sleep(1000); // run for 1000 ms (1 second)

        // Stop motors
        MainHardwareMap.FrontLeftDTM.setPower(0);
        MainHardwareMap.FrontRightDTM.setPower(0);
        MainHardwareMap.RearLeftDTM.setPower(0);
        MainHardwareMap.RearRightDTM.setPower(0);

        telemetry.addLine("Autonomous Complete");
        telemetry.update();
    }

}

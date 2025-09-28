package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MainHardwareMap.FrontLeftDTM;
import static org.firstinspires.ftc.teamcode.MainHardwareMap.FrontRightDTM;
import static org.firstinspires.ftc.teamcode.MainHardwareMap.RearLeftDTM;
import static org.firstinspires.ftc.teamcode.MainHardwareMap.RearRightDTM;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "MainTeleOp", group = "Linear Op Mode")
public class MainTeleOp extends LinearOpMode {

    // Gamepad tracking
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    //Sets Default Motor Speed When Operating
    double DriveSpeed = 1;

    @Override
    public void runOpMode() {
        // === Initialize hardware (MUST be called) ===
        MainHardwareMap.init(hardwareMap);

        // Initialize telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for start
        waitForStart();

        // Run until stop
        while (opModeIsActive()) {

            // Copy gamepad states
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Gets Encoder Position

            int FLMPosition = FrontLeftDTM.getCurrentPosition();
            int FRMPosition = FrontRightDTM.getCurrentPosition();
            int RLMPosition = RearLeftDTM.getCurrentPosition();
            int RRMPosition = RearRightDTM.getCurrentPosition();

            // Driver Control

                /* Denominator is just the way to put the motor power in a number from 0 to 1
               by getting the full equation and multiplying it by Drive Speed which has a default value
                of 1 it stays the same. By The toggle setting drive speed to .5 it puts motor power in half.
             */
            double Denominator = (Math.max(Math.abs(gamepad1.left_stick_y) + Math.abs(-gamepad1.left_stick_x) + Math.abs(-gamepad1.right_stick_x), 1));

            //Mechanum Drive Variables
            double FrontLeftPower = ((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / Denominator * DriveSpeed);
            double FrontRightPower = ((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / Denominator * DriveSpeed);
            double RearLeftPower = ((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / Denominator * DriveSpeed);
            double RearRightPower = ((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / Denominator * DriveSpeed);

            FrontLeftDTM.setPower(FrontLeftPower);
            FrontRightDTM.setPower(FrontRightPower);
            RearLeftDTM.setPower(RearLeftPower);
            RearRightDTM.setPower(RearRightPower);


            //Makes Speed Toggle to Lower speed when button is held

            if (gamepad1.left_trigger > 0.0){
                DriveSpeed = .5;
            }
            else {
                DriveSpeed = 1;
            }

            //Operator Control

            // Update telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("FLDTM", FLMPosition);
            telemetry.addData("FRDTM", FRMPosition);
            telemetry.addData("RLDTM", RLMPosition);
            telemetry.addData("RRDTM", RRMPosition);
            telemetry.addData("DriveSpeed", DriveSpeed);
            telemetry.update();
        }

        // === Stop vision when done (optional) ===
        //MainHardwareMap.closeVision();
    }
}
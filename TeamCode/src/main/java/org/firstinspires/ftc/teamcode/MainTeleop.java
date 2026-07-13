package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MainHardwareMap.*;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MainTeleop", group = "Learning")
public class MainTeleop extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        MainHardwareMap.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        if (opModeIsActive()){
            //DriveTrain
            double strafeFix = 1.1;
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * strafeFix; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            FrontLeftDTM.setPower(frontLeftPower);
            FrontRightDTM.setPower(backLeftPower);
            RearLeftDTM.setPower(frontRightPower);
            RearRightDTM.setPower(backRightPower);

        }
}
}

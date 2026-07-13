package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MainHardwareMap {
    public static DcMotor FrontLeftDTM;
    public static DcMotor FrontRightDTM;
    public static DcMotor RearLeftDTM;
    public static DcMotor RearRightDTM;

    public static void init(HardwareMap hwMap) {

        FrontLeftDTM = hwMap.get(DcMotorEx.class, "FrontLeftDTM");
        FrontRightDTM = hwMap.get(DcMotorEx.class, "FrontRightDTM" );
        RearLeftDTM = hwMap.get(DcMotorEx.class, "RearLeftDTM" );
        RearRightDTM = hwMap.get(DcMotorEx.class, "RearRightDTM" );

        DcMotor[] driveMotors = {FrontLeftDTM, FrontRightDTM, RearLeftDTM, RearRightDTM};

        for (DcMotor motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

}

}

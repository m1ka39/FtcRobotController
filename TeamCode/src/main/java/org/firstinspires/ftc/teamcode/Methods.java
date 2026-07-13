package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.MainHardwareMap.*;

public class Methods {

    public void forwards(){
        MainHardwareMap.FrontLeftDTM.setPower(1.0);
        MainHardwareMap.FrontRightDTM.setPower(1.0);
        MainHardwareMap.RearLeftDTM.setPower(1.0);
        MainHardwareMap.RearRightDTM.setPower(1.0);

    }
}

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//Import Color Sensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;


import java.util.List;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MainHardwareMap {

    // === Drivetrain Motors ===
    public static DcMotor FrontLeftDTM;
    public static DcMotor FrontRightDTM;
    public static DcMotor RearLeftDTM;
    public static DcMotor RearRightDTM;

    // Vision implements
    private static AprilTagProcessor aprilTag ;
    private static VisionPortal visionPortal;
    private static final boolean USE_WEBCAM = true;
    // Color Sensor
    static NormalizedColorSensor Sensor1;

    // === Init Method ===

    public static void init(HardwareMap hwMap) {
        // Map Motors
        FrontLeftDTM = hwMap.get(DcMotor.class, "FrontLeft");
        FrontRightDTM = hwMap.get(DcMotor.class, "FrontRight");
        RearLeftDTM = hwMap.get(DcMotor.class, "RearLeft");
        RearRightDTM = hwMap.get(DcMotor.class, "RearRight");

        // Map Servos

        // Map Servos
        Sensor1 = hwMap.get(NormalizedColorSensor.class, "TTColorSensor");
        // Motor Directions

        FrontLeftDTM.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightDTM.setDirection(DcMotorSimple.Direction.FORWARD);
        RearLeftDTM.setDirection(DcMotorSimple.Direction.REVERSE);
        RearRightDTM.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset Encoders
        FrontLeftDTM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDTM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftDTM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightDTM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run with Encoders
        FrontLeftDTM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDTM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeftDTM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRightDTM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Zero Power Behavior
        FrontLeftDTM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDTM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeftDTM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRightDTM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Default Power
        FrontLeftDTM.setPower(0);
        FrontRightDTM.setPower(0);
        RearLeftDTM.setPower(0);
        RearRightDTM.setPower(0);

        // Default Target Position
        FrontLeftDTM.setTargetPosition(0);
        FrontRightDTM.setTargetPosition(0);
        RearLeftDTM.setTargetPosition(0);
        RearRightDTM.setTargetPosition(0);
        //Vision Portal
        initAprilTag(hwMap);
    }


    private static void initAprilTag(HardwareMap hwMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        aprilTag.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    // Telemetry
    public static void telemetryAprilTag(Telemetry telemetry) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up)");
        telemetry.addLine("PRY = Pitch, Roll & Yaw");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        telemetry.update();
    }

    // Get detections
    public static List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    // Vision lifecycle
    public static void stopVision() {
        if (visionPortal != null) visionPortal.stopStreaming();
    }

    public static void resumeVision() {
        if (visionPortal != null) visionPortal.resumeStreaming();
    }

    public static void closeVision() {
        if (visionPortal != null) visionPortal.close();
    }
}

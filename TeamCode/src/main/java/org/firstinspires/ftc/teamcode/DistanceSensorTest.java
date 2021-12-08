package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name = "dSensor", group = "Linear Opmode")
public class DistanceSensorTest extends LinearOpMode{
    HardwareMap hwMap;
    public DistanceSensor dSensor;

@Override
    public void runOpMode() {
    telemetry.addData("Status", "Running");
    telemetry.update();
    dSensor = hardwareMap.get(DistanceSensor.class, "dSensor");
    waitForStart();
    while(opModeIsActive()) {
        double value = dSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("dSensor Distance", value);
        telemetry.update();
    }
}
}
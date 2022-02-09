package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name = "dSensorTest", group = "Linear Opmode")
public class DistanceSensorTest extends LinearOpMode{
    HardwareMap hwMap;
    public DistanceSensor dSensorRight;
    public DistanceSensor dSensorLeft;
    public DcMotor arm;

@Override
    public void runOpMode() {
    telemetry.addData("Status", "Running");
    telemetry.update();
    dSensorRight = hardwareMap.get(DistanceSensor.class, "dSensorRight");
    dSensorLeft = hardwareMap.get(DistanceSensor.class, "dSensorLeft");
    arm = hardwareMap.get(DcMotor.class, "leftArm");
    waitForStart();
    while(opModeIsActive()) {
        double value = dSensorRight.getDistance(DistanceUnit.CM);
        double value2 = dSensorLeft.getDistance(DistanceUnit.CM);
        telemetry.addData("dSensorRight Distance", value);
        telemetry.addData("dSensorLeft Distance", value2);
        telemetry.addData("Arm Ticks", arm.getCurrentPosition());
        telemetry.update();
    }
}
}
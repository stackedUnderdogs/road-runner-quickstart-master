package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.differentiation.JacobianFunction;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

public class BlueAutonomousBlocks extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor elevator = null;
    public DcMotor towerSpinner = null;
    public DcMotor intake = null;
    public DistanceSensor dSensor = null;
    public TouchSensor tSensorTop = null;
    public TouchSensor tSensorBottom = null;
    HardwareMap hwMap = null;
    BNO055IMU imu;
    Orientation angles;
    public SampleMecanumDrive drive;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;


        dSensor = hwMap.get(DistanceSensor.class, "dSensor");
        tSensorTop = hwMap.get(TouchSensor.class, "tSensorTop");
        tSensorBottom = hwMap.get(TouchSensor.class, "tSensorBottom");
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        elevator = hwMap.get(DcMotor.class, "elevator");
        intake = hwMap.get(DcMotor.class, "intake");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.FORWARD); //FIX
        intake.setDirection(DcMotorSimple.Direction.FORWARD); //FIX

        this.stopDriving();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive = new SampleMecanumDrive(ahwMap);
        drive.setPoseEstimate(new Pose2d());
    }

    public void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveForward(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void moveBackward(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
    }

    public void runIntake() {
        ElapsedTime intakeTime = new ElapsedTime();
        intakeTime.reset();
        while(opModeIsActive() && intakeTime.seconds() < 1) { //FIX may take more/less than 1 seconds
            intake.setPower(1); //FIX don't know if works
        }
    }

    public void stopIntake() {

        intake.setPower(0);
    }

    public void moveToDuck() {
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(12).build()); //FIX check if moves all the way to duck
    }

    public int detectDuck() {
        //if 1 then duck in middle
        //if 0 duck on right
        //if 2 duck on left
        if (dSensor.getDistance(DistanceUnit.INCH) < 27) { //fix to correct
            return 1;
        }
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(10).build());

        if (dSensor.getDistance(DistanceUnit.INCH) < 27) { //fix to correct
            return 0;
        }
        return 2;
    }

    public void moveElevatorOneStep() {
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setTargetPosition(1120); //FIX change if reaches
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stopElevator();
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stopElevator() {

        while(!elevator.isBusy()) {
            elevator.setPower(0);
        }
    }

    public void moveToShippingHub(int duck) {
        if (duck == 1) {
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(24).build()); //FIX might not go to shipping hub (from middle)
        }
        else if(detectDuck() == 0 || detectDuck() == 2) {
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(14).build()); //FIX might not go to shipping hub (from right)
        }
    }

    public void placeInFirstLevel() {

        moveElevatorOneStep();
    }

    public void placeInSecondLevel() {
        moveElevatorOneStep();
        moveElevatorOneStep();
    }

    public void placeInThirdLevel() {
        moveElevatorOneStep();
        moveElevatorOneStep();
        moveElevatorOneStep();
    }

    public void placeInitCube() {

         int duck = detectDuck();
         moveToShippingHub(duck);
        if(duck == 0) { //duck on right
            placeInFirstLevel();
        }
        else if(duck == 1) { //duck in middle
            placeInSecondLevel();
        }
        else if(duck == 2) { //duck on left
            placeInThirdLevel();
        }

    }

    public void pickUpBlock() {
        if(dSensor.getDistance(DistanceUnit.INCH) < 1) { //FIX change to correct distance from sensor to blocks
            moveForward(1); //FIX may cause robot to move forever
        }
        else {
            stopDriving();
        }
        runIntake();
        stopIntake();


    }

    public void getBlock() {
        //strafe
        Pose2d currentPosition = drive.getPoseEstimate();
        pickUpBlock();
        //FIX move backward until back at currentPosition
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(currentPosition.vec()).build()); //FIX might not be correct pose


    }

    void resetEncoders(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void runOpMode() {
        init(hwMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //make sure imu is mounted flat on robot
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



        waitForStart();
        moveToDuck(); //go to ducks
        placeInitCube(); //detect which duck, move to shipping hub, and place cube

    }
}
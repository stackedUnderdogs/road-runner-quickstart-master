package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//TILE DIMENSION 24 x 24 INCH
//1st level tick count = 190
//2nd level tick count = 280
//3rd level tick count = 395
//about 6 inches away
//right dSensor to intake = 4 inches
//left dSensor to intake =
//fails right after intake
@Config
@Autonomous(name = "BlueAutoDucks", group = "Autonomous")
public class BlueAutonomousDucks extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor elevator = null;
    public DcMotor towerSpinner = null;
    public DcMotor intake = null;
    public DistanceSensor dSensorRight = null;
    public DistanceSensor dSensorLeft = null;
    public DistanceSensor dSensorIntake = null;
    HardwareMap hwMap = null;
    BNO055IMU imu;
    Orientation angles;
    public SampleMecanumDrive drive;
    public Pose2d lineUpWithOpening;
    //public DigitalChannel intakeLight; //ADD BACK
    public Pose2d startingPose;
    public Vector2d shippingHubPose;

    public void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public int detectDuck() {
        //if 1 then duck in middle
        //if 0 duck on right
        //if 2 duck on left
        if (dSensorRight.getDistance(DistanceUnit.CM) < 31) { //fix to correct
            //return 1;
            return 1;
        }
        if(dSensorLeft.getDistance(DistanceUnit.CM) < 31) {
            //return 2;
            return 2;
        }
        //return 0;
        return 0;
        //drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(8).build()); //FIX was 8.5
    }

    public void moveToShippingHub(int duck) {
        if (duck == 1 || duck == 2 || duck == 0) {
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeLeft(22.5)
                    .forward(9)
                    .build()); //FIX might not go to shipping hub (from middle)
            //drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(3).build());
        }
    }

    public void resetElevator() {
        elevator.setTargetPosition(-5);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void placeInFirstLevel() {

        elevator.setTargetPosition(190);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(-1);
        /*while (Math.abs(elevator.getCurrentPosition() - 190) > 5){
        }
        outtakeBlock();*/
        Runnable thread = new Thread(new WaitTask2(elevator, 190, this));
        thread.run();
    }

    public void placeInSecondLevel() {

        elevator.setTargetPosition(285);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(-1);
        /*while (Math.abs(elevator.getCurrentPosition() - 285) > 5){
        }
        outtakeBlock();*/
        Runnable thread = new Thread(new WaitTask2(elevator, 285, this));
        thread.run();
    }

    public void placeInThirdLevel() {

        elevator.setTargetPosition(395);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(-1);
        /*while (Math.abs(elevator.getCurrentPosition() - 395) > 5){
        }
        outtakeBlock();*/
        Runnable thread = new Thread(new WaitTask2(elevator, 395, this));
        thread.run();
    }

    public void placeInitCube() {
        startingPose = drive.getPoseEstimate();
        int duck = detectDuck();
        moveToShippingHub(duck);
        shippingHubPose = drive.getPoseEstimate().vec();
        if(duck == 2) { //duck on left
            placeInFirstLevel();

        }
        else if(duck == 1) { //duck in middle
            placeInSecondLevel(); // correct one
            //placeInFirstLevel(); //check for error
        }
        else if(duck == 0) { //duck on right
            placeInThirdLevel();
            //placeInFirstLevel();
        }

    }

    public void intakeBlock() {
        resetElevator();
        intake.setDirection(DcMotor.Direction.REVERSE);
        //intake.setPower(1);
        while(dSensorIntake.getDistance(DistanceUnit.INCH) > 1.5) { //FIX
            intake.setPower(1);
            /*if(dSensorIntake.getDistance(DistanceUnit.INCH) < 1) {
                intakeLight.setState(true);
            }*/
        } //ADD BACK
        //intake.setPower(0);
    }

    public void outtakeBlock() {
        intake.setDirection(DcMotor.Direction.FORWARD);
        //intake.setPower(1);
        while(dSensorIntake.getDistance(DistanceUnit.INCH) < 5) { //FIX
            intake.setPower(0.5);
            /*if(dSensorIntake.getDistance(DistanceUnit.INCH) > 2) {
                intakeLight.setState(false);
            }*/
        } //ADD BACK
        intake.setPower(0);
        resetElevator();
    }

    public void pickUpBlock() {
        /*double moveBackInches = 0;
        if(dSensor.getDistance(DistanceUnit.INCH) < 1) { //FIX change to correct distance from sensor to blocks
            moveBackInches = dSensor.getDistance(DistanceUnit.INCH);
            moveForward(1); //FIX may cause robot to move forever
        }
        else {
            stopDriving();
        }
        //stopDriving();
        runIntake();
        stopIntake();
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(moveBackInches).build());*/
        double moveBackInches = 0;
        if(dSensorRight.getDistance(DistanceUnit.INCH) > 1) {
            moveBackInches = dSensorRight.getDistance(DistanceUnit.INCH);
        }
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).strafeRight(4).forward(moveBackInches).build());
        //runIntake();
        //stopIntake();
        intakeBlock();
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).back(moveBackInches).strafeLeft(4).build());
        //drive.setPoseEstimate(drive.getPoseEstimate()); if doesnt work then add
        //drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).back(moveBackInches).strafeLeft(4).build());
    }

    public void moveToBlock() {
        double moveBackInchesFull = dSensorRight.getDistance(DistanceUnit.INCH);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(moveBackInchesFull - 2).build());
        pickUpBlock();
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(moveBackInchesFull - 2).build());
    }

    public void getBlock() {
        if(dSensorRight.getDistance(DistanceUnit.INCH) < 31 ) {
            //strafe
            //Pose2d currentPosition = drive.getPoseEstimate();
            moveToBlock();

            //FIX move backward until back at currentPosition
            //drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(currentPosition.vec()).build()); //FIX might not be correct pose
        } else {         //Strafe right until lines up with an object
            Vector2d currentVector = drive.getPoseEstimate().vec();
            Pose2d currentPose = drive.getPoseEstimate();
            while(dSensorRight.getDistance(DistanceUnit.INCH) > 31) { //FIX WILL CAUSE ISSUE
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(1).build());
                //drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(1).build());
            }
            moveToBlock();
            /*if(!(drive.getPoseEstimate().vec().distTo(currentVector) < 1)) { //in correct spot
                new TrajectoryBuilder(drive.followTrajectory(lineToConstantHeading(currentVector)).build());
            }*/
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineToConstantHeading(currentVector).build());
        }

    }

    public void goToCarousel() {
        drive.turn(Math.toRadians(90));
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).strafeLeft(24).back(40).build());
    }

    public void resetBackToWareHouse() {
        drive.turn(Math.toRadians(82)); //FIX MIGHT TURN WRONG WAY
        //drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(6/*FIX*/).forward(48/*FIX*/).build());
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(59).build());
    }

    public void parkBot() {
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).strafeRight(28.5).back(5).build());
        //drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(5).build());
    }

    public void moveToBarcode() {
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(5.5).build());
    }

    public void runCarousel(double power, long milliseconds) {
        towerSpinner.setPower(power);
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        towerSpinner.setPower(0);
    }

    public void runOpMode() {
        telemetry.addData("Status", "Running");
        telemetry.update();
        dSensorRight = hardwareMap.get(DistanceSensor.class, "dSensorRight");
        dSensorLeft = hardwareMap.get(DistanceSensor.class, "dSensorLeft");
        dSensorIntake = hardwareMap.get(DistanceSensor.class, "dSensorIntake"); //ADD BACK
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        elevator = hardwareMap.get(DcMotor.class, "leftArm");
        towerSpinner = hardwareMap.get(DcMotor.class, "towerSpinner");
        intake = hardwareMap.get(DcMotor.class, "intakeSystem");
        //intakeLight = hardwareMap.get(DigitalChannel.class, "intakeLight"); //ADD BACK
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.FORWARD); //FIX
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD); //FIX
        towerSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        this.stopDriving();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //intakeLight.setMode(DigitalChannel.Mode.OUTPUT);

        //make sure imu is mounted flat on robot
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



        waitForStart();
        while(opModeIsActive()){
            sleep(5000);
            moveToBarcode();
            placeInitCube(); //detect which duck, move to shipping hub, and place cube
            goToCarousel();
            runCarousel(0.5, 6000);
            parkBot();
            break;
            //park
        }
    }
}

class WaitTask2 implements Runnable {

    private DcMotor motor;
    private int target;
    private BlueAutonomousDucks auto;

    public WaitTask2(DcMotor motor, int target, BlueAutonomousDucks auto){
        this.motor = motor;
        this.target = target;
        this.auto = auto;
    }

    @Override
    public void run(){
        while (Math.abs(motor.getCurrentPosition() - target) > 5){

        }
        auto.outtakeBlock();
    }
}
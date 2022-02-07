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
//1st level tick count = -190
//2nd level tick count = -280
//3rd level tick count = -395
//about 6 inches away
//right dSensor to intake = 4 inches
//left dSensor to intake =
@Config
@Autonomous(name = "BlueAutoWarehouse", group = "Autonomous")
public class BlueAutonomousBlocks extends LinearOpMode {
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
    //public LED intakeLight;
    public Pose2d startingPose;
    public Vector2d shippingHubPose;

   /* public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;


        dSensorRight = hardwareMap.get(DistanceSensor.class, "dSensorRight");
        dSensorLeft = hardwareMap.get(DistanceSensor.class, "dSensorLeft");
        dSensorIntake = hardwareMap.get(DistanceSensor.class, "dSensorIntake");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        intake = hardwareMap.get(DcMotor.class, "intake");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.FORWARD); //FIX
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD); //FIX

        this.stopDriving();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive = new SampleMecanumDrive(ahwMap);
        drive.setPoseEstimate(new Pose2d());
    } */

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
        if (dSensorRight.getDistance(DistanceUnit.INCH) < 22) { //fix to correct
            return 1;
        }
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(8).build()); //FIX was 8.5

        if (dSensorRight.getDistance(DistanceUnit.INCH) < 22) { //fix to correct
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
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(28)
                    .forward(16)
                    .build()); //FIX might not go to shipping hub (from middle)
            //drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(3).build());
        }
        else if(detectDuck() == 0 || detectDuck() == 2) {
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(19.5).build()); //FIX might not go to shipping hub (from right)
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(16).build());
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
        Runnable thread = new Thread(new WaitTask(elevator, 190, this));
        thread.run();
    }

    public void placeInSecondLevel() {

        elevator.setTargetPosition(280);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(-1);
        /*while (Math.abs(elevator.getCurrentPosition() - 280) > 5){
        }
        outtakeBlock();*/
        Runnable thread = new Thread(new WaitTask(elevator, 280, this));
        thread.run();
    }

    public void placeInThirdLevel() {

        elevator.setTargetPosition(395);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(-1);
        /*while (Math.abs(elevator.getCurrentPosition() - 395) > 5){
        }
        outtakeBlock();*/
        Runnable thread = new Thread(new WaitTask(elevator, 395, this));
        thread.run();
    }

    public void placeInitCube() {
        startingPose = drive.getPoseEstimate();
         int duck = detectDuck();
         moveToShippingHub(duck);
         shippingHubPose = drive.getPoseEstimate().vec();
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

    public void intakeBlock() {
        resetElevator();
        intake.setDirection(DcMotor.Direction.REVERSE);
        //intake.setPower(1);
        while(dSensorIntake.getDistance(DistanceUnit.INCH) > 1.5) { //FIX
            intake.setPower(1);
            /*if(dSensorIntake.getDistance(DistanceUnit.INCH) < 1) {
                intakeLight.enableLight(true);
            }*/
        } //ADD BACK
        //intake.setPower(0);
    }

    public void outtakeBlock() {
        intake.setDirection(DcMotor.Direction.FORWARD);
        //intake.setPower(1);
        while(dSensorIntake.getDistance(DistanceUnit.INCH) < 10) { //FIX
            intake.setPower(1);
            /*if(dSensorIntake.getDistance(DistanceUnit.INCH) > 2) {
                intakeLight.enableLight(false);
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
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(moveBackInches).strafeLeft(4).build());
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
            }
            moveToBlock();
            /*if(!(drive.getPoseEstimate().vec().distTo(currentVector) < 1)) { //in correct spot
                new TrajectoryBuilder(drive.followTrajectory(lineToConstantHeading(currentVector)).build());
            }*/
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineToConstantHeading(currentVector).build());
        }

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
    public void goToWarehouse() {
        Pose2d pose = drive.getPoseEstimate();
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(10).build()); //FIX back was 16
        drive.turn(Math.toRadians(90)); //FIX IDK why negative
        //straighten robot
        if(pose.getHeading() + Math.toRadians(90) != drive.getPoseEstimate().getHeading()){
            drive.turn((pose.getHeading() + Math.toRadians(90) - drive.getPoseEstimate().getHeading()));
        }
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(47).build()); //FIX strafe not needed, make sure goes over the barrier
        //FIX go over barrier then reset turn
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(3).build()); //FIX implement a vector
        if(pose.getHeading() + Math.toRadians(90) != drive.getPoseEstimate().getHeading()){
            drive.turn((pose.getHeading() + Math.toRadians(90) - drive.getPoseEstimate().getHeading()));
        }
    }

    public void positionInWarehouse() {
//        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(12/*FIX*/).forward(6/*FIX*/).build());
//        drive.turn(-Math.PI/2);
//        lineUpWithOpening = drive.getPoseEstimate();
//        lineUpWithOpeningHeading = drive.getPoseEstimate().getHeading(); This is if placing in shared hub
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(6/*FIX*/).build()); //This is if placing in alliance hub
    }

    public void moveToOpening() {
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading(lineUpWithOpening).build());
    }

    public void goToSharedShippingHub() {
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(6/*FIX*/).forward(12/*FIX*/).strafeRight(6/*FIX*/).forward(1/*FIX might not need*/).build());
    }

    public void resetBackToWareHouse() {
        drive.turn(Math.toRadians(-90)); //FIX MIGHT TURN WRONG WAY
        //drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(6/*FIX*/).forward(48/*FIX*/).build());
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(59).build());
    }

    public void getBlockAndPlaceBlockInSharedHubThenReset() {
        //FIX need to move to block
        /*getBlock();
        moveToOpening();
        drive.turn(Math.PI/2);
        goToSharedShippingHub();
        placeInFirstLevel(); //FIX may be to low/high
        resetBackToWareHouse();*/ //This is for placing in shared hub
        //straighten robot
        if(drive.getPoseEstimate().getHeading() != startingPose.getHeading() + Math.toRadians(90)) {
            drive.turn((startingPose.getHeading() + Math.toRadians(90)) - drive.getPoseEstimate().getHeading());
        }
        getBlock();
        if(drive.getPoseEstimate().getHeading() != startingPose.getHeading() + Math.toRadians(90)) {
            drive.turn((startingPose.getHeading() + Math.toRadians(90)) - drive.getPoseEstimate().getHeading());
        }
        //drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(6).back(48).build());
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(5).build());
        //FIX go over barrier implement a vec
        if(drive.getPoseEstimate().getHeading() != startingPose.getHeading() + Math.toRadians(90)) {
            drive.turn((startingPose.getHeading() + Math.toRadians(90)) - drive.getPoseEstimate().getHeading());
        }
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(47).build());
        drive.turn(Math.toRadians(-90)); // FIX MIGHT TURN WRONG WAY
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(10).build());
        //OR FIX
        //drive.turn(Math.toRadians(-90);
        //drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(shippingHubPose).build());
        placeInThirdLevel();
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(10).build());
        resetBackToWareHouse();
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
        intake = hardwareMap.get(DcMotor.class, "intakeSystem");
        //intakeLight = hardwareMap.get(LED.class, "intakeLight"); //ADD BACK
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.FORWARD); //FIX
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD); //FIX

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

        //make sure imu is mounted flat on robot
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



        waitForStart();
        while(opModeIsActive()){
        //moveToDuck(); //go to ducks
        placeInitCube(); //detect which duck, move to shipping hub, and place cube
        goToWarehouse(); //go to warehouse from shipping hub
        //positionInWarehouse(); //face blocks
        /*
        pickUpBlock(); //FIX need to move to block
        moveToOpening(); //go to opening between storage and shared shipping hub
        drive.turn(Math.PI); //turn around
        goToSharedShippingHub(); //go to shared hub
        placeInFirstLevel(); //place in shared hub FIX might be too low or high
        resetBackToWareHouse();// go back and pick up cube
        getBlock();
        moveToOpening();
        */
        getBlockAndPlaceBlockInSharedHubThenReset(); //do for as much time as you have
        getBlockAndPlaceBlockInSharedHubThenReset();
        //park
    }
}
}

class WaitTask implements Runnable {

    private DcMotor motor;
    private int target;
    private BlueAutonomousBlocks auto;

    public WaitTask(DcMotor motor, int target, BlueAutonomousBlocks auto){
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
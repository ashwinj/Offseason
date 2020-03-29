//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled

public class Auto_Methods extends LinearOpMode {
    private BNO055IMU imu;
    public Hardware robot = new Hardware();
    private double gearRatio = 2;
    private double wheelDiameter = 4;
    private double encoderCnts = 560; //counts per one rotation of output shaft

    public void initRobot() {
        robot.initTeleOpNOIMU(hardwareMap);

        //initializing imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        telemetry.addLine("wait for gyro calibration"); //telling user to wait for gyro to be calibrated
        telemetry.update();

        //waiting for gyro to calibrate
        while (!imu.isGyroCalibrated()) {
            //waiting
        }

        telemetry.addData("gyro status :: ", imu.getCalibrationStatus().toString()); //returning gyro status (ready to start)
        telemetry.update();
        waitForStart();
    }

    //defining methods for returning imu reading

    //getting angle based on initial angle
    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;

        return heading; //right :: [0, -180] -- left :: [0, 180]
    }

    private double sigmoid(double error) {
        if (error < 40) {
            return (error / (40 / .3)) + .05;
        }
        return .3;
    }


    //moving forward distance (m) with power [0, 1]
    public void forward(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int counts = (int) ((distance / (wheelDiameter * Math.PI)) * (encoderCnts / gearRatio));


        //setting all motors to go forward (positive)

        double factor = 1;

        if (power >= .5) {
            factor = 10;
        }

        robot.backLeftMotor.setPower(power / factor);
        robot.backRightMotor.setPower(power / factor);
        robot.frontRightMotor.setPower(power / factor);
        robot.frontLeftMotor.setPower(power / factor);

        while (counts >= robot.backLeftMotor.getCurrentPosition()) {
            if (factor > 1) {
                factor -= .2;
            }
            robot.backLeftMotor.setPower(power / factor);
            robot.backRightMotor.setPower(power / factor);
            robot.frontRightMotor.setPower(power / factor);
            robot.frontLeftMotor.setPower(power / factor);
        }


        //setting all motor powers to 0 (stopping)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(500);
    }

    //moving backward distance (m) with power [0, 1]
    public void backward(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int counts = -(int) ((distance / (wheelDiameter * Math.PI)) * (encoderCnts / gearRatio));

        //setting all motors to go forward (positive)

        double factor = -1;

        if (power >= .5) {
            factor = -10;
        }

        robot.backLeftMotor.setPower(power / factor);
        robot.backRightMotor.setPower(power / factor);
        robot.frontRightMotor.setPower(power / factor);
        robot.frontLeftMotor.setPower(power / factor);

        while (counts <= robot.backLeftMotor.getCurrentPosition()) {
            if (factor < -1) {
                factor += .2;
            }
            robot.backLeftMotor.setPower(power / factor);
            robot.backRightMotor.setPower(power / factor);
            robot.frontRightMotor.setPower(power / factor);
            robot.frontLeftMotor.setPower(power / factor);
        }


        //setting all motor powers to 0 (stopping)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(500);
    }

    //turning right angle (deg) with power [0, 1]
    public void right(double degrees) {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double initAngle = getHeading(); //angle that the robot is at when it starts

        //wait until angle turned is >= angle inputted
        while (initAngle - getHeading() <= degrees) {


            //setting left motors to go forward (positive power)
            robot.backLeftMotor.setPower(sigmoid(degrees - (initAngle - getHeading())));
            robot.frontLeftMotor.setPower(sigmoid(degrees - (initAngle - getHeading())));

            //setting right motors to go backward (negative power)
            robot.backRightMotor.setPower(-sigmoid(degrees - (initAngle - getHeading()))); //-
            robot.frontRightMotor.setPower(-sigmoid(degrees - (initAngle - getHeading()))); //-
        }

        //setting motor value to 0 (stop)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    //turning left angle (deg) with power [0, 1]
    public void left(double degrees) {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double initAngle = getHeading(); //angle that the robot is at when it starts

        //wait until angle turned is >= angle inputted
        while (getHeading() - initAngle <= degrees) {

            //setting right motors to go backward (negative power)
            robot.backRightMotor.setPower(sigmoid(degrees - (getHeading() - initAngle)));
            robot.frontRightMotor.setPower(sigmoid(degrees - (getHeading() - initAngle)));

            //setting left motors to go forward (positive power)
            robot.backLeftMotor.setPower(-sigmoid(degrees - (getHeading()-initAngle)));
            robot.frontLeftMotor.setPower(-sigmoid(degrees - (getHeading()-initAngle)));
        }

        //setting motor value to 0 (stop)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void adjust(double degrees) {
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double initAngle = getHeading(); //angle that the robot is at when it starts

        //wait until angle turned is >= angle inputted
        //wait until angle turned is >= angle inputted
        while (getHeading() > degrees + 1 || getHeading() < degrees - 1) {
            if (getHeading() >= degrees) {
                while (getHeading() >= degrees) {

                    //setting left motors to go forward (positive power)
                    robot.backLeftMotor.setPower(sigmoid(degrees - (initAngle - getHeading())));
                    robot.frontRightMotor.setPower(-sigmoid(degrees - (initAngle - getHeading()))); //-
                    robot.backRightMotor.setPower(-sigmoid(degrees - (initAngle - getHeading()))); //-
                    robot.frontLeftMotor.setPower(sigmoid(degrees - (initAngle - getHeading())));

                    //setting right motors to go backward (negative power)

                }
            } else {
                while (getHeading() <= degrees) {
                    //setting right motors to go backward (negative power)
                    robot.backRightMotor.setPower(sigmoid(degrees - (getHeading() - initAngle)));
                    robot.frontLeftMotor.setPower(-sigmoid(degrees - (getHeading() - initAngle)));
                    robot.backLeftMotor.setPower(-sigmoid(degrees - (getHeading() - initAngle)));
                    robot.frontRightMotor.setPower(sigmoid(degrees - (getHeading() - initAngle)));
                    //setting left motors to go forward (positive power)
                }
            }
        }

        //setting motor value to 0 (stop)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //strafing left distance (m) with power [0, 1]
    public void strafeLeft(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int counts = (int) ((distance / (wheelDiameter * Math.PI)) * (encoderCnts / gearRatio));

        //setting all motors to go forward (positive)

        double factor = 1;

        if (power >= .5) {
            factor = 10;
        }

        robot.backLeftMotor.setPower(power / factor);
        robot.frontLeftMotor.setPower(-power / factor);
        robot.frontRightMotor.setPower(power / factor);
        robot.backRightMotor.setPower(-power / factor);

        while (counts >= robot.backLeftMotor.getCurrentPosition()) {
            if (factor > 1) {
                factor -= .2;
            }
            robot.backLeftMotor.setPower(power / factor);
            robot.frontLeftMotor.setPower(-power / factor);
            robot.frontRightMotor.setPower(power / factor);
            robot.backRightMotor.setPower(-power / factor);
        }


        //setting all motor powers to 0 (stopping)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(500);

    }

    //strafing right distance (m) with power [0, 1]
    public void strafeRight(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int counts = -(int) ((distance / (wheelDiameter * Math.PI)) * (encoderCnts / gearRatio));

        //setting all motors to go forward (positive)

        double factor = 1;

        if (power >= .5) {
            factor = 10;
        }

        robot.backLeftMotor.setPower(-power / factor);
        robot.frontLeftMotor.setPower(power / factor);
        robot.frontRightMotor.setPower(-power / factor);
        robot.backRightMotor.setPower(power / factor);


        while (counts <= robot.backLeftMotor.getCurrentPosition()) {
            if (factor > 1) {
                factor -= .2;
            }
            robot.backLeftMotor.setPower(-power / factor);
            robot.frontLeftMotor.setPower(power / factor);
            robot.frontRightMotor.setPower(-power / factor);
            robot.backRightMotor.setPower(power / factor);
        }


        //setting all motor powers to 0 (stopping)
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        sleep(500);

    }

    //moving vertical lift up a duration (seconds) with power [0,1]
    public void liftUp(double power, int time) {
        robot.leftLiftMotor.setPower(power);
        robot.rightLiftMotor.setPower(power);
        sleep(time);
        robot.rightLiftMotor.setPower(.1);
        robot.leftLiftMotor.setPower(.1);
    }

    //dropping vertical lift by setting power to 0
    public void liftDrop() {
        robot.rightLiftMotor.setPower(-.1);
        robot.leftLiftMotor.setPower(-.1);
        while (!robot.rightLimitSwitch.getState()) {

        }
        robot.rightLiftMotor.setPower(0);
        robot.leftLiftMotor.setPower(0);
    }

    public void liftOut(double power, int time) {
        robot.leftExtensionServo.setPower(power);
        robot.rightExtensionServo.setPower(-power);
        sleep(time);
        robot.leftExtensionServo.setPower(.1);
    }

    public void liftIn(double power, int time) {
        robot.leftExtensionServo.setPower(-power);
        robot.rightExtensionServo.setPower(power);
        sleep(time);
        robot.leftExtensionServo.setPower(0);
    }

    public void liftDown(double power, int time) {
        robot.leftLiftMotor.setPower(-power);
        robot.rightLiftMotor.setPower(-power);
        sleep(time);
        robot.rightLiftMotor.setPower(0);
        robot.leftLiftMotor.setPower(0);
    }

    public void intake(){
        robot.leftIntake.setPower(1);
        robot.rightIntake.setPower(.75);
        robot.intake2.setPower(.7);
        robot.intake3.setPower(.7);
    }

    public void intake2() {
        robot.leftIntake.setPower(1);
        robot.rightIntake.setPower(1);
        robot.intake2.setPower(.7);
        robot.intake3.setPower(.7);
    }

    public void outtake(){
        robot.leftIntake.setPower(-1);
        robot.rightIntake.setPower(-1);
        robot.intake2.setPower(-.7);
        robot.intake3.setPower(-.7);
    }

    public void stopIntake(){
        robot.leftIntake.setPower(0);
        robot.rightIntake.setPower(0);
        robot.intake2.setPower(0);
        robot.intake3.setPower(0);
    }

    public void clawOpen() {
        robot.clawServo.setPosition(0);
    }

    public void isBlockIn() {
        if (robot.blockSensor.getDistance(DistanceUnit.INCH)<2){
            robot.clawServo.setPosition(.5);
        }
        robot.clawServo.setPosition(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
    }
}


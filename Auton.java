package org.firstinspires.ftc.teamcode.ReplaceOdo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Autonomous(name="Autonomous", group="Pushbot")
public class Auton extends LinearOpMode {

    public DcMotorEx motorFrontRight = null;
    public DcMotorEx motorFrontLeft = null;
    public DcMotorEx  motorBackRight = null;
    public DcMotorEx motorBackLeft = null;  
    public BNO055IMU imu;

    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;


    public double lastError = 0;

    public double motorSpeed = 0.8;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "mbl");
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight = hardwareMap.get(DcMotorEx.class, "mbr");
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "mfr");
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "mfl");
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        waitForStart();
        while(opModeIsActive()) {
            PIDTune(90);

        }
    }
    public void displacePosition(int mfl, int mfr, int mbr, int mbl, double power){
        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition()+ mfl);
        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition()+ mbl);
        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition()+ mfr);
        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition()+ mbr);

        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

        motorFrontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);



    }
    public void displacePosition(int position, double power) {
        displacePosition(position, position, position, position, power);
    }

    public void displacePosition(double speed) {

        motorFrontLeft.setPower(speed);
        motorBackLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);
    }


    public void newHeading(double referenceAngle){
        useEncoders();
        double power = PIDControl(Math.toRadians(referenceAngle), imu.getAngularOrientation().firstAngle);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(-power);
        motorBackRight.setPower(-power);
        toPosition();
    }

    public void relativeXMovement(int position){
        double power = PIDControlD(position, motorFrontLeft.getCurrentPosition());
        displacePosition(position, position, position, position, power);
    }
    //this code assumes that the right side of the relative robot is positive and the left is negative
    public void relativeYMovement(int position){
        newHeading(90);
        double power = PIDControlD(position, motorFrontLeft.getCurrentPosition());
        displacePosition(position, position, position, position, power);
        newHeading(270);
    }
    //assumes that right is positive and left is negative relative to the robot
    public void YMovementMecanum(int position){
        double power = PIDControlD(position, motorFrontLeft.getCurrentPosition());
        displacePosition(position, -position, position, -position, power);
    }

    //this will first move in the X direction, then the Y, then to the correct heading (Meant for tank drive)
    public void attemptSingleFunctionMovement(int xMovement, int yMovement, double referenceAngle){
        relativeXMovement(xMovement);
        relativeYMovement(yMovement);
        newHeading(referenceAngle);
    }
    //meant for tank drive, turns to the angle of displacement then moves to displacement target then fixes its angle
    public void attemptSingleFunctionMovementButBetter(int xMovement, int yMovement, double referenceAngle){
        //get the angle with atan2 and distance with the formula sqrt of X^2+Y^2
        double angle = Math.atan2(yMovement,xMovement);
        double magnitude = Math.sqrt(xMovement * xMovement + yMovement * yMovement);
        int roundedMagnitude = (int)Math.round(angle);

        newHeading(angle);
        relativeXMovement(roundedMagnitude);
        double finalAngle = Math.toRadians(referenceAngle) - angle;
        newHeading(Math.toRadians(finalAngle));
    }
    public void attemptSingleFunctionMovementButBetterAndAlsoForMecanum(int xMovement, int yMovement, double referenceAngle){
        relativeXMovement(xMovement);
        YMovementMecanum(yMovement);
        newHeading(referenceAngle);
    }
    public void attemptSingleFunctionMovementButBetterAndAlsoForMecanumAndImproved(int xMovement, int yMovement, double referenceAngle) {
        double angle = Math.atan2(yMovement, xMovement); // Radians
        double magnitude = Math.sqrt(xMovement * xMovement + yMovement * yMovement);

        // Calculate initial motor powers
        double powerSin = magnitude * Math.sin(angle + Math.PI / 4);
        double powerCos = magnitude * Math.cos(angle + Math.PI / 4);

        // Normalize motor powers if any is greater than 1.0
        double maxPower = Math.max(Math.abs(powerCos), Math.abs(powerSin));
        if (maxPower > 1.0) {
            powerSin /= maxPower;
            powerCos /= maxPower;
        }


        double frontLeftPower = PIDControlD(powerSin, motorFrontLeft.getCurrentPosition());
        double frontRightPower = PIDControlD(powerCos, motorFrontRight.getCurrentPosition());
        double backLeftPower = PIDControlD(powerCos, motorBackLeft.getCurrentPosition());
        double backRightPower = PIDControlD(powerSin, motorBackRight.getCurrentPosition());

        // Set motor powers
        motorFrontLeft.setPower(frontLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackLeft.setPower(backLeftPower);
        motorBackRight.setPower(backRightPower);
    }
    public void distance(int Expectedx, int Expectedy, int currentX, int currentY){
        
    }

    public void useEncoders(){
        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void toPosition(){
        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void PIDTune(int referenceAngle){
        telemetry.addData("Target IMU Angle", referenceAngle);
        telemetry.addData("Current IMU Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        double power = PIDControl(referenceAngle, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        newHeading(referenceAngle);
        telemetry.update();
    }

    public double PIDControl(double reference, double state){
        double error = angleWrap(reference - state);
        integralSum += error * timer.seconds();
        double derivative  = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    public double PIDControlD(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative  = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }


    public double angleWrap(double radians){
        while (radians > Math.PI){
            radians -= 2* Math.PI;
        } while (radians < Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }






}

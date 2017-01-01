package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class MecanumAaron extends OpMode
{
    private double timeLog = System.currentTimeMillis();
    //Declares name of DcMotors
    DcMotor motorRF;            //Gamepad 1: right stick and left stick
    DcMotor motorLF;            //Gamepad 1: right stick and left stick
    DcMotor motorRB;            //Gamepad 1: right stick and left stick
    DcMotor motorLB;            //Gamepad 1: right stick and left stick

    MecanumDrive drive;

    OnOffSwitch servoLRTime;

    OnOffSwitch servoUTime;

    BNO055IMU imu;

    LightSensor light;

    Servo servoL;
    Servo servoR;
    Servo servoT;

    DcMotor motorCol;
    DcMotor motorPop;

    DcMotor motorLiftR;
    DcMotor motorLiftL;

    boolean servoLRUp = false;

    boolean servoTUp = false;

    private int upNdown = 0;
    @Override
    public void init()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        light = hardwareMap.lightSensor.get("light");
        light.enableLed(true);

        drive = new MecanumDrive(hardwareMap.dcMotor.get("motorRF"), hardwareMap.dcMotor.get("motorLF"), hardwareMap.dcMotor.get("motorRB"), hardwareMap.dcMotor.get("motorLB"));

        motorCol = hardwareMap.dcMotor.get("collector");
        motorPop = hardwareMap.dcMotor.get("popper");

        motorLiftR = hardwareMap.dcMotor.get("liftCapballR");
        motorLiftL = hardwareMap.dcMotor.get("liftCapballL");

        servoL = hardwareMap.servo.get("servoL");
        servoR = hardwareMap.servo.get("servoR");
        servoT = hardwareMap.servo.get("servoT");

        double originalangle = imu.getAngularOrientation().firstAngle;
    }
    //@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
    @Override
    public void loop()
    {
        timeLog = System.currentTimeMillis();

        //light.enableLed(true);
        double smallMove = 1;
        if(gamepad1.right_bumper)
        {
            smallMove = 0.2;
        }
        //Moving
        double gyr = 0;
        if(gamepad1.b)
        {
            gyr = imu.getAngularOrientation().firstAngle;
        }

        double y = -gamepad1.left_stick_y*smallMove;
        double x = gamepad1.left_stick_x*smallMove;
        double turn = gamepad1.right_stick_x;
        drive.move(x, y, turn, gyr + 270);

        //Alt movment
            if(gamepad2.dpad_up)
            {
                drive.move(0, 1, 0, 0);
            }
            else if(gamepad2.dpad_down)
            {
                drive.move(0, -1, 0, 0);
            }
            else if(gamepad2.dpad_left)
            {
                drive.move(-1, 0, 0, 0);
            }
            else if(gamepad2.dpad_right)
            {
                drive.move(1, 0, 0, 0);
            }
        //Collector
            motorCol.setPower(-gamepad2.right_trigger);
            //motorUL.setPower(lift);
            //motorUR.setPower(lift);
        //Popper
            motorPop.setPower(gamepad2.left_trigger*1);

        //Lift Capball R=1 L=2 T=3
            if((upNdown >= 0 && -gamepad2.right_stick_y < -0.5)||(gamepad2.left_bumper && -gamepad2.right_stick_y < -0.5))
            {
                upNdown -= 1;
                motorLiftR.setPower(-1);
                motorLiftL.setPower(-1);
            }
            else if((upNdown <= 10000 && -gamepad2.right_stick_y > 0.5)||(gamepad2.left_bumper && -gamepad2.right_stick_y < -0.5))
            {
                upNdown += 1;
                motorLiftR.setPower(1);
                motorLiftL.setPower(1);
            }
            else
            {
                motorLiftL.setPower(0);
                motorLiftR.setPower(0);
            }

        telemetry.addData("unNdown num ", upNdown);

        telemetry.addData("Light Num", light.getLightDetected());
        //ServoBall
        if(gamepad2.x)
        {
            //servoLRTime.setTime(timeLog, 1000);
            servoLRUp = true;
        }
        if(gamepad2.a)
        {
            //servoLRTime.setTime(timeLog, 1000);
            servoLRUp = false;
            servoTUp = false;
        }

        if(servoLRUp)
        {
            servoL.setPosition(90);
            servoR.setPosition(0);
        }
        else
        {
            servoL.setPosition(0);
            servoR.setPosition(90);
        }
        //Servo TopBall
        if(gamepad2.y)
        {
            ;
            servoTUp = true;
        }

        if(servoTUp)
        {
            servoT.setPosition(0);
        }
        else
        {
            servoT.setPosition(180);
        }

    }

    public void mecaMovement(double degrees, double power)
    {
        double radians = Math.toRadians(degrees - 135);
        motorLF.setPower(power * Math.cos(radians));
        motorLB.setPower(power * Math.sin(radians));
        motorRF.setPower(power * Math.sin(radians));
        motorRB.setPower(power * Math.cos(radians));
    }

    @Override
    public void stop()
    {

    }

}
package org.firstinspires.ftc.robotcontroller.external.samples;
import org.firstinspires.ftc.robotcontroller.external.samples.MecanumDrive;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;

import java.util.ArrayList;

public class MecanumWheels extends OpMode
{

    //Declares name of DcMotors
    DcMotor motorRF;            //Gamepad 1: right stick and left stick
    DcMotor motorLF;            //Gamepad 1: right stick and left stick
    DcMotor motorRB;            //Gamepad 1: right stick and left stick
    DcMotor motorLB;            //Gamepad 1: right stick and left stick

    MecanumDrive drive;

    //GyroSensor gyro;

    //SensorAdafruitRGB color;
    //SensorAdafruitIMU IMU;
    //I2cDevice gyro;

    //GyroSensor aGyro;

    Servo buttonServo;

    DcMotor motorCol;
    DcMotor motorPop;
    @Override
    public void init()
    {
        drive = new MecanumDrive(hardwareMap.dcMotor.get("motorRF"), hardwareMap.dcMotor.get("motorLF"), hardwareMap.dcMotor.get("motorRB"), hardwareMap.dcMotor.get("motorLB"));

        motorCol = hardwareMap.dcMotor.get("collector");
        motorPop = hardwareMap.dcMotor.get("popper");

        //IMU = hardwareMap.deviceInterfaceModule.get("a");
        //motorCol.setDirection(DcMotor.Direction.REVERSE);
        //motorUR.setDirection(DcMotor.Direction.REVERSE);
        //color = (SensorAdafruitRGB) hardwareMap.colorSensor.get("color");
        buttonServo = hardwareMap.servo.get("bServo");
        //gyro = hardwareMap.i2cDevice.get("aGyro");
        //aGyro = hardwareMap.gyroSensor.get("aGyro");
        //DeviceInterfaceModule
    }
    //@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
    @Override
    public void loop()
    {
        double smallMove = 1;
        if(gamepad1.right_bumper)
        {
            smallMove = 0.2;
        }
        //Moving
        double gyr = 0;
            double y = -gamepad1.left_stick_y*smallMove;
            double x = gamepad1.left_stick_x*smallMove;
            double turn = gamepad1.right_stick_x;
            drive.move(x, y, turn, gyr);

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
        //Servo Button
            double button = 180;
            if(gamepad1.x)
            {
                button = -180;
            }
            else if(gamepad1.b)
            {
                button = 0;
            }
            else if(gamepad1.a)
            {
                button = 360;
            }
            buttonServo.setPosition(button);
        //Collector
            motorCol.setPower(gamepad2.right_trigger);
            //motorUL.setPower(lift);
            //motorUR.setPower(lift);
        //Popper
            motorPop.setPower(gamepad2.left_trigger*1);

        //color
        //DbgLog.msg("MY_DEBUG - x button was released!");
        if(gamepad1.dpad_up || gamepad2.y)
        {
            drive.move(Math.sqrt(-1),99,-0,-12341);
        }
//        motorLF.setPower(magnitude * Math.cos(radians) + turn);
//        motorLB.setPower(magnitude * Math.sin(radians) + turn);
//        motorRF.setPower(magnitude * Math.sin(radians) - turn);
//        motorRB.setPower(magnitude * Math.cos(radians) - turn);

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
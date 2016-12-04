package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class MecanumAaron extends OpMode
{

    //Declares name of DcMotors
    DcMotor motorRF;            //Gamepad 1: right stick and left stick
    DcMotor motorLF;            //Gamepad 1: right stick and left stick
    DcMotor motorRB;            //Gamepad 1: right stick and left stick
    DcMotor motorLB;            //Gamepad 1: right stick and left stick

    MecanumDrive drive;

    //GyroSensor gyro;
    LightSensor light;
    //SensorAdafruitRGB color;
    //SensorAdafruitIMU IMU;
    //I2cDevice gyro;

    //GyroSensor aGyro;

    //Servo buttonServo;

    DcMotor motorCol;
    DcMotor motorPop;
    @Override
    public void init()
    {
        drive = new MecanumDrive(hardwareMap.dcMotor.get("motorRF"), hardwareMap.dcMotor.get("motorLF"), hardwareMap.dcMotor.get("motorRB"), hardwareMap.dcMotor.get("motorLB"));

        motorCol = hardwareMap.dcMotor.get("collector");
        motorPop = hardwareMap.dcMotor.get("popper");

        light = hardwareMap.lightSensor.get("light");
        //IMU = hardwareMap.deviceInterfaceModule.get("a");
        //motorCol.setDirection(DcMotor.Direction.REVERSE);
        //motorUR.setDirection(DcMotor.Direction.REVERSE);
        //color = (SensorAdafruitRGB) hardwareMap.colorSensor.get("color");

        //gyro = hardwareMap.i2cDevice.get("aGyro");
        //aGyro = hardwareMap.gyroSensor.get("aGyro");
        //DeviceInterfaceModule
    }
    //@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
    @Override
    public void loop()
    {
        light.enableLed(true);
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
        //Collector
            motorCol.setPower(gamepad2.right_trigger);
            //motorUL.setPower(lift);
            //motorUR.setPower(lift);
        //Popper
            motorPop.setPower(gamepad2.left_trigger*1);

        telemetry.addData("Light  ", light.getLightDetected());

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
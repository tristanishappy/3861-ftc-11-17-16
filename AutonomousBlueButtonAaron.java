package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.ModernRoboticsI2cColorSensor2;

public class AutonomousBlueButtonAaron extends LinearOpMode
{
    //Declares name of DcMotors
    DcMotor motorRF;            //Gamepad 1: right stick and left stick
    DcMotor motorLF;            //Gamepad 1: right stick and left stick
    DcMotor motorRB;            //Gamepad 1: right stick and left stick
    DcMotor motorLB;            //Gamepad 1: right stick and left stick

    ModernRoboticsI2cColorSensor2 colorx;

    LightSensor light;

    I2cDevice buttonColor;

    DcMotor motorCol;
    DcMotor motorPop;
    private double lift = 0;

    Servo servoL;
    Servo servoR;

    @Override
    public void runOpMode() throws InterruptedException
    {
        I2cDevice buttonColor = hardwareMap.i2cDevice.get("color");
        colorx = new ModernRoboticsI2cColorSensor2(buttonColor.getI2cController(),buttonColor.getPort());

        servoL = hardwareMap.servo.get("servoL");
        servoR = hardwareMap.servo.get("servoR");

        motorRF = hardwareMap.dcMotor.get("motorRF");//sets DcMotors to type of motor
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        //buttonColor = hardwareMap.colorSensor.get("color");

        light = hardwareMap.lightSensor.get("light");
        motorCol = hardwareMap.dcMotor.get("collector");
        motorPop = hardwareMap.dcMotor.get("popper");

        motorCol.setDirection(DcMotor.Direction.REVERSE);

        //motorUR.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        motorRF.setDirection(DcMotor.Direction.REVERSE);

        light.enableLed(true);
        //buttonColor.enableLed(false);
        //Movement



        motorPop.setPower(1);
        sleep(1500);
        motorPop.setPower(0);
        sleep(100);

        mecaMovement(0,0);
        sleep(100);

        mecaMovement(0,0.5);
        sleep(500);

        movement(1,-1);//turn right
        sleep(660);

        mecaMovement(270,0.3);
        sleep(850);

        mecaMovement(0,0.5);
        sleep(500);

        mecaMovement(45, 0.5);
        sleep(3870);

        mecaMovement(270,0.45);//nah
        sleep(400);


        mecaMovement(90, 0.35);
        while (light.getLightDetected() <= 0.4)
        {
            //telemetry.addData("Light  ", light.getLightDetected());
            sleep(1);
        }
        //mecaMovement(90,0.5);
        //sleep(300);

        mecaMovement(90,0.5);//nah?
        sleep(170);

        telemetry.addData("Colornumber: ",colorx.colorNumber());
        telemetry.addData("Red: ",colorx.red());
        telemetry.addData("Blue: ",colorx.blue());

        if(colorx.colorNumber() == 0)
        {
            mecaMovement(270,0.5);
            sleep(200);
        }

        if(colorx.colorNumber() == 0)
        {
            mecaMovement(0,0.5);
            sleep(200);
        }

        mecaMovement(0,0);
        sleep(1000);

        while(colorx.colorNumber() == 0)
        {
            //telemetry.addData("Light  ", light.getLightDetected());
            mecaMovement(0, 0);
            sleep(1);
        }

        if(colorx.colorNumber() == 3 || colorx.colorNumber() == 2 || colorx.colorNumber() == 1)
        {
            movement(-1,1);//Right
            sleep(10);
            mecaMovement(0,0.35);
            sleep(1300);
        }
        else if (colorx.colorNumber() >= 8){
            movement(1,-1);//Right
            sleep(10);
            mecaMovement(90,0.35);
            sleep(400);
            mecaMovement(0,0.35);
            sleep(1300);
        }
        mecaMovement(180, 0.35);
        sleep(400);

        mecaMovement(90, 0.4);
        sleep(200);

        mecaMovement(90,0.5);
        sleep(1000);
        mecaMovement(0,0.35);
        sleep(1800);
        mecaMovement(180, 0.4);
        sleep(355);


        mecaMovement(90, 0.35);
        while (light.getLightDetected() <= 0.4)
        {
            //telemetry.addData("Light  ", light.getLightDetected());
            sleep(1);
        }
        mecaMovement(90,0.35);
        sleep(180);

        mecaMovement(0,0);//nah?
        sleep(300);

        telemetry.addData("Colornumber: ",colorx.colorNumber());
        mecaMovement(0,0);
        sleep(1000);

        while(colorx.colorNumber() == 0)
        {
            sleep(1);
        }

        if(colorx.colorNumber() == 3 || colorx.colorNumber() == 2)
        {
            movement(1,-1);//Right
            sleep(10);
            mecaMovement(0,0.35);
            sleep(1300);
            mecaMovement(180, 0.35);
            sleep(600);
            mecaMovement(90,0.35);
            sleep(250);
        }
        else if(colorx.colorNumber() == 9 || colorx.colorNumber() == 10){
            movement(-1,1);//Right
            sleep(10);
            mecaMovement(90,0.35);
            sleep(250);
            mecaMovement(0,0.35);
            sleep(1300);
            mecaMovement(180, 0.35);
            sleep(400);

        }
        else
        {
            mecaMovement(180, 0.35);
            sleep(400);
        }
        mecaMovement(180,0);
        sleep(200);

        movement(1,-1);//turn right
        sleep(1080);
        mecaMovement(0,0.5);
        sleep(700);
        mecaMovement(0,0);
        sleep(100);

        motorCol.setPower(0.7);
        sleep(1500);
        motorPop.setPower(1);
        sleep(1500);
        motorPop.setPower(0);
        sleep(100);

        mecaMovement(0, 1);
        sleep(2000);

        telemetry.addLine("Did it work???");
    }

    public void mecaMovement(double degrees, double power)
    {
        double radians = Math.toRadians(degrees+135);
        motorLF.setPower(power * Math.cos(radians));
        motorLB.setPower(power * Math.sin(radians));
        motorRF.setPower(power * Math.sin(radians));
        motorRB.setPower(power * Math.cos(radians));
    }
    public void movement(double powerL, double powerR)
    {
        motorLF.setPower(powerL);
        motorLB.setPower(powerL);
        motorRF.setPower(powerR);
        motorRB.setPower(powerR);
    }
}
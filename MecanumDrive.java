package org.firstinspires.ftc.robotcontroller.external.samples;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by Tristan on 9/26/2016.
 */
public class MecanumDrive
{
    DcMotor motorRF;            //Gamepad 1: right stick and left stick
    DcMotor motorLF;            //Gamepad 1: right stick and left stick
    DcMotor motorRB;            //Gamepad 1: right stick and left stick
    DcMotor motorLB;            //Gamepad 1: right stick and left stick
    public MecanumDrive(DcMotor mRF, DcMotor mLF, DcMotor mRB, DcMotor mLB)
    {
        motorRF = mRF;//sets DcMotors to type of motor
        motorLF = mLF;
        motorRB = mRB;
        motorLB = mLB;

        motorRB.setDirection(DcMotor.Direction.REVERSE);
        motorRF.setDirection(DcMotor.Direction.REVERSE);
    }

    public void move(double x, double y, double turn, double gyro)
    {
        double magnitude = Math.sqrt(y * y + x * x);
        double radians = Math.atan2(y,x) - (3*Math.PI/4 + Math.toRadians(gyro));

        if (Math.abs(magnitude) + Math.abs(turn) > 1)
        {
            magnitude = magnitude/(Math.abs(magnitude) + Math.abs(turn));
            turn = Math.signum(turn) * (1 - Math.abs(magnitude));
        }
        motorLF.setPower(magnitude * Math.cos(radians) + turn);
        motorLB.setPower(magnitude * Math.sin(radians) + turn);
        motorRF.setPower(magnitude * Math.sin(radians) - turn);
        motorRB.setPower(magnitude * Math.cos(radians) - turn);
    }
}

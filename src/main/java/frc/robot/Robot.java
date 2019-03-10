package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Calculates empirical robot trackwidth in feet by spinning robot.
 */
public class Robot extends TimedRobot {

    CANSparkMax leftMotor, rightMotor;
    CANEncoder leftEncoder, rightEncoder;
    ADXRS450_Gyro gyro;

    final double TICKS_PER_REV = 1; // spark returns rotations
    final double WHEEL_DIAMETER = (6.0 / 12.0); // feet
    final double GEAR_RATIO = 15.32; // set to 1 if rotations are from output shaft

    @Override
    public void robotInit() {

        // Motor controllers
        leftMotor = new CANSparkMax(1, MotorType.kBrushless);
        leftMotor.setInverted(false);
        leftMotor.setIdleMode(IdleMode.kBrake);

        CANSparkMax leftSlave1 = new CANSparkMax(2, MotorType.kBrushless);
        leftSlave1.setInverted(false);
        leftSlave1.setIdleMode(IdleMode.kBrake);
        leftSlave1.follow(leftMotor);

        CANSparkMax leftSlave2 = new CANSparkMax(3, MotorType.kBrushless);
        leftSlave2.setInverted(false);
        leftSlave2.setIdleMode(IdleMode.kBrake);
        leftSlave2.follow(leftMotor);

        rightMotor = new CANSparkMax(4, MotorType.kBrushless);
        rightMotor.setInverted(false);
        rightMotor.setIdleMode(IdleMode.kBrake);

        CANSparkMax rightSlave1 = new CANSparkMax(5, MotorType.kBrushless);
        rightSlave1.setInverted(false);
        rightSlave1.setIdleMode(IdleMode.kBrake);
        rightSlave1.follow(leftMotor);

        CANSparkMax rightSlave2 = new CANSparkMax(6, MotorType.kBrushless);
        rightSlave2.setInverted(false);
        rightSlave2.setIdleMode(IdleMode.kBrake);
        rightSlave2.follow(leftMotor);

        // Encoders
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        // Gyro
        gyro = new ADXRS450_Gyro();

    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() {
        // Reset
        gyro.reset();
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    @Override
    public void autonomousPeriodic() {

        double rotations = Math.abs(gyro.getAngle()) / 360.0;

        if(rotations < 10) {
            leftMotor.set(1.0);
            rightMotor.set(-1.0);
        } else {
            leftMotor.set(0);
            rightMotor.set(0);
            double nativeDistance = (Math.abs(leftEncoder.getPosition()) +
                    Math.abs(rightEncoder.getPosition())) / 2.0;
            double distanceInFeet = nativeDistance * (1.0 / TICKS_PER_REV) * 
                (1.0 / GEAR_RATIO) * WHEEL_DIAMETER * Math.PI;
            double trackwidth = distanceInFeet / (rotations * Math.PI);
            SmartDashboard.putNumber("Trackwidth (ft)", trackwidth);
        }

    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testPeriodic() {
    }

}

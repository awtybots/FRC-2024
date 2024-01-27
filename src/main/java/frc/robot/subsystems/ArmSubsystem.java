package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Presets;
import frc.robot.RobotContainer;
import frc.robot.util.math.Convert;
import frc.robot.util.math.Convert.Encoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax mLeftArmMotor;
    private CANSparkMax mRightArmMotor;

    

    public final RelativeEncoder mRightArmEncoder;

    private final SparkPIDController mRightArmPIDController;

    public double armHeight;
    private final double kArmGearRatio = 1.0 / (216.0);

    public ArmSubsystem() {

        mLeftArmMotor = new CANSparkMax(Arm.kRightArmMotorId, MotorType.kBrushless);
        mRightArmMotor = new CANSparkMax(Arm.kLeftArmMotorId, MotorType.kBrushless);
        // mLeftArmMotor.restoreFactoryDefaults();

        mRightArmMotor.restoreFactoryDefaults();

        // Current limit
        mLeftArmMotor.setSmartCurrentLimit(Arm.kCurrentLimit);
        mRightArmMotor.setSmartCurrentLimit(Arm.kCurrentLimit);

        mRightArmMotor.setInverted(true);
        mLeftArmMotor.follow(mRightArmMotor, true);

        armHeight = Arm.initialHeight;

        mRightArmEncoder = mRightArmMotor.getEncoder();

        mRightArmPIDController = mRightArmMotor.getPIDController();

        mRightArmPIDController.setP(Arm.kP);
        mRightArmPIDController.setI(Arm.kI);
        mRightArmPIDController.setD(Arm.kD);
        mRightArmPIDController.setIZone(Arm.kIz);
        mRightArmPIDController.setFF(Arm.kFF);
        mRightArmPIDController.setOutputRange(-0.6, 0.6);

        // mRightArmPIDController.setFeedbackDevice(mRightArmEncoder);

    }

    public void setRotation(double value) {
        armHeight = value;
    }

    public void setDegrees(double degrees) {
        double encoderunits =
                Convert.angleToEncoderPos(degrees, kArmGearRatio, Encoder.RevRelativeEncoder);
        armHeight = encoderunits;
    }

    public double getAngle() {
        final double rawRevs = mRightArmEncoder.getPosition();
        final double theta =
                Convert.encoderPosToAngle(rawRevs, kArmGearRatio, Encoder.RevRelativeEncoder);
        return Arm.startingAngle - theta;
    }

    public void resetEncoderValue() {
        armHeight = 0;
        mRightArmEncoder.setPosition(armHeight);
    }

    public void drive(double pct) {

        // MathUtil.clamp(armHeight, ArmConstants.minimumHeight, getMaximumRotation());
        armHeight += pct * Arm.armConversion;
    }

    public boolean isFinished() {
        return Math.abs(mRightArmEncoder.getPosition() - armHeight) < Presets.ArmThreshold;
    }

    @Override
    public void periodic() {
        mRightArmPIDController.setReference(armHeight, CANSparkMax.ControlType.kPosition, 0);
        // SmartDashboard.putNumber(
        //         "Arm Error",
        //         Convert.encoderPosToAngle(
        //                 mRightArmEncoder.getPosition() - armHeight, kArmGearRatio,
        // Encoder.RevRelativeEncoder));
        // SmartDashboard.putNumber("Arm angle", -(this.getAngle() - Arm.startingAngle));
    }

    public void stop() {
        mRightArmMotor.set(0);
    }
}

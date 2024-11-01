package frc.robot.Subsytems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Pivot extends SubsystemBase {
    private static Pivot instance;
    private final DigitalInput irBeamBreak; // irBeamBreak  connected to DIO port 0 


    private final TalonFX pivotMotor1;
    private final TalonFX pivotMotor2;
    private final Follower followerControl;
    private final MotionMagicDutyCycle motionMagicControl;
    private final TalonFXConfiguration pivotMotorConfig;  

    public Pivot() {
        pivotMotor1 = new TalonFX(Constants.PivotConstants.kPivotMotor1CanId);
        pivotMotor2 = new TalonFX(Constants.PivotConstants.kPivotMotor2CanId);
        irBeamBreak = new DigitalInput(Constants.PivotConstants.kPivotIrBeamBreakPort);
        pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.withSlot0(Constants.PivotConstants.kPivotConfiguration);
        pivotMotorConfig.withMotionMagic(Constants.PivotConstants.kPivotMotionMagic);
        pivotMotor1.getConfigurator().apply(pivotMotorConfig);
        pivotMotor2.getConfigurator().apply(pivotMotorConfig);

        pivotMotor1.setNeutralMode(NeutralModeValue.Brake);
        pivotMotor2.setNeutralMode(NeutralModeValue.Brake);

        motionMagicControl = new MotionMagicDutyCycle(0, true, 0, 0, false, false, false);
        followerControl = new Follower(pivotMotor1.getDeviceID(), true);
        pivotMotor2.setControl(followerControl);
    }

    public static synchronized Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }

    public void setPosition(double position) {
        pivotMotor1.setControl(motionMagicControl.withPosition(position));
    }

    public double getPosition() {
        return pivotMotor1.getPosition().getValueAsDouble();
    }

    public boolean isAtPositionSetpoint(double position) {
        return Math.abs(pivotMotor1.getPosition().getValueAsDouble() - position) < Constants.PivotConstants.kPivotErrorMargin;
    }

    // Commands
    public Command rotateToPosition(double position) {
        return this.run(() -> setPosition(position)).until(() -> isAtPositionSetpoint(position));
    }

    public Command stow() {
        return this.run(() -> setPosition(Constants.PivotConstants.kStowPosition))
                .until(() -> isAtPositionSetpoint(Constants.PivotConstants.kStowPosition));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", pivotMotor1.getPosition().getValueAsDouble());
        Logger.recordOutput("Pivot Position", getPosition());
    }
    public boolean isPivotBeamBroken() {
        return !irBeamBreak.get(); // irBeamBreak is active-low
    }
    public void resetPivotEncoder() {
        pivotMotor1.setPosition(0);
    }
}
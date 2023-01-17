package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
// import frc.lib.util.CTREModuleState; 
import frc.lib.util.SwerveModuleConstants;

// import com.ctre.phoenix.motorcontrol.ControlMode;   
// import com.ctre.phoenix.motorcontrol.DemandType;   
// import com.ctre.phoenix.motorcontrol.can.TalonFX;   
import com.ctre.phoenix.sensors.CANCoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.controller.PIDController;       
// import edu.wpi.first.wpilibj.geometry.Rotation2d;           
// import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;   
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
// import com.revrobotics.AbsoluteEncoder;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    // private TalonFX mAngleMotor; /*FIXME [DONE] Replaced with NEO motors */
    // private TalonFX mDriveMotor; /*FIXME [DONE] Replaced with NEO motors */
    private CANSparkMax m_drivingSparkMax;
    private CANSparkMax m_turningSparkMax;
    
    private final CANCoder angleEncoder;
    private final RelativeEncoder m_drivingEncoder;
    private final RelativeEncoder m_turningEncoder;

    private final SparkMaxPIDController m_drivingPIDController;
    private final SparkMaxPIDController m_turningPIDController;
    private final PIDController m_CancoderTurningController;

    private SwerveModulePosition m_modulePosition;

    private final Rotation2d chassisOffsetAngular = Rotation2d.fromRadians(0.0);
    // private double m_chassisAngularOffset = 0; /*FIXME [DONE] Do we need this?????*/

    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */ 
        // mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        m_turningSparkMax = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        // mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        m_drivingSparkMax = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_drivingSparkMax.restoreFactoryDefaults();
        m_turningSparkMax.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = m_turningSparkMax.getEncoder();
        m_drivingPIDController = m_drivingSparkMax.getPIDController();
        m_turningPIDController = m_turningSparkMax.getPIDController();
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        m_drivingEncoder.setPositionConversionFactor(Constants.Swerve.kDrivingEncoderPositionFactor);
        m_drivingEncoder.setVelocityConversionFactor(Constants.Swerve.kDrivingEncoderVelocityFactor);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_turningEncoder.setPositionConversionFactor(Constants.Swerve.kTurningEncoderPositionFactor);
        m_turningEncoder.setVelocityConversionFactor(Constants.Swerve.kTurningEncoderVelocityFactor);

        /*FIXME SHOULD WE INCLUDE THIS?? */
        m_turningPIDController.setPositionPIDWrappingEnabled(true);
        m_turningPIDController.setPositionPIDWrappingMinInput(Constants.Swerve.kTurningEncoderPositionPIDMinInput);
        m_turningPIDController.setPositionPIDWrappingMaxInput(Constants.Swerve.kTurningEncoderPositionPIDMaxInput);
        
        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_drivingPIDController.setP(Constants.Swerve.driveKP);
        m_drivingPIDController.setI(Constants.Swerve.driveKI);
        m_drivingPIDController.setD(Constants.Swerve.driveKD);
        m_drivingPIDController.setFF(Constants.Swerve.driveKF);
        m_drivingPIDController.setOutputRange(Constants.Swerve.kDrivingMinOutput,
            Constants.Swerve.kDrivingMaxOutput);

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_turningPIDController.setP(Constants.Swerve.angleKP);
        m_turningPIDController.setI(Constants.Swerve.angleKI);
        m_turningPIDController.setD(Constants.Swerve.angleKD);
        m_turningPIDController.setFF(Constants.Swerve.angleKF);
        m_turningPIDController.setOutputRange(Constants.Swerve.kTurningMinOutput,
            Constants.Swerve.kTurningMaxOutput);

        m_drivingSparkMax.setIdleMode(Constants.Swerve.drivingIdleMode);
        m_turningSparkMax.setIdleMode(Constants.Swerve.steeringIdleMode);
        m_drivingSparkMax.setSmartCurrentLimit(Constants.Swerve.kDrivingMotorCurrentLimit);
        m_turningSparkMax.setSmartCurrentLimit(Constants.Swerve.kTurningMotorCurrentLimit);


        // chassisOffsetAngular = (moduleConstants.angleOffset);
        m_turningEncoder.setPosition(angleEncoder.getAbsolutePosition());

        // m_chassisAngularOffset = angleOffset; /*FIXME [DONE] DO WE NEED THIS? */
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);

        m_CancoderTurningController = new PIDController(
            Constants.Swerve.angleKP,
            Constants.Swerve.angleKI,
            Constants.Swerve.angleKD
    );

        // lastAngle = getState().angle;

        m_modulePosition = new SwerveModulePosition();

         // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_drivingSparkMax.burnFlash();
        m_turningSparkMax.burnFlash();
    }

    public void reset(){
        m_turningEncoder.setPosition(angleEncoder.getAbsolutePosition());
      }
    
      public void stop(){
        m_drivingSparkMax.set(0);
        m_turningSparkMax.set(0);
      }
    
      public void updatePosition(){
        m_modulePosition.angle = getSteerAngle();
        m_modulePosition.distanceMeters = getDriveDistance();
      }
    
      public SwerveModulePosition getPosition(){
        updatePosition();
        return m_modulePosition;
      }
    
      public Rotation2d getSteerAngle(){
        double angle = m_turningEncoder.getPosition() - this.angleOffset.getRadians();
        return Rotation2d.fromRadians(angle); 
      }
    
      public double getDriveDistance(){
        return m_drivingEncoder.getPosition();
      }
    
      public SwerveModuleState getState(){
        return new SwerveModuleState(m_drivingSparkMax.getEncoder().getVelocity(), getSteerAngle());
      }
    
      public void setDesiredState(SwerveModuleState state,boolean isOpenLoop) {
        // System.out.println("Pre Optimize: " + state.speedMetersPerSecond);
        state = SwerveModuleState.optimize(state, getSteerAngle());
        // System.out.println("Post Optimize: " + state.speedMetersPerSecond);
        // System.out.println("Setting: " + (state.speedMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond));
        m_drivingSparkMax.set(state.speedMetersPerSecond / Constants.Swerve.maxSpeed);
        m_turningSparkMax.set(
          m_CancoderTurningController.calculate(
            m_turningEncoder.getPosition(), 
            state.angle.getRadians() + this.angleOffset.getRadians())
        );
      }
    
      public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRadians(angleEncoder.getAbsolutePosition()); 
      }
    
    //   @Override
    //   public void periodic() {
    //     // This method will be called once per scheduler run
      
    















// /**
//    * Returns the current state of the module.
//    *
//    * @return The current state of the module.
//    */
//   public SwerveModuleState getState() {
//     // Apply chassis angular offset to the encoder position to get the position
//     // relative to the chassis.
//     return new SwerveModuleState(m_drivingEncoder.getVelocity(),
//         new Rotation2d(m_turningEncoder.getPosition() - this.angleOffset.getRadians())); /*FIXME HOPE THIS WORKS */
//   }
  
//   /**
//    * Returns the current position of the module.
//    *
//    * @return The current position of the module.
//    */
// //   public SwerveModulePosition getPosition() {
// //     // Apply chassis angular offset to the encoder position to get the position
// //     // relative to the chassis.
// //     return new SwerveModulePosition(
// //         m_drivingEncoder.getPosition(),
// //         new Rotation2d(m_turningEncoder.getPosition() - this.angleOffset.getRadians()));
// //   }

  
//   /**
//    * Sets the desired state for the module.
//    *
//    * @param desiredState Desired state with speed and angle.
//    */
//   public void setDesiredState(SwerveModuleState desiredState,boolean isOpenLoop) {
//     // Apply chassis angular offset to the desired state.
//     SwerveModuleState correctedDesiredState = new SwerveModuleState();
//     correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
//     correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(this.angleOffset.getRadians()));

//     // Optimize the reference state to avoid spinning further than 90 degrees.
//     SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
//         new Rotation2d(m_turningEncoder.getPosition()));

//     // Command driving and turning SPARKS MAX towards their respective setpoints.
//     m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
//     m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

//     m_desiredState = desiredState;
//   }

//   /** Zeroes all the SwerveModule encoders. */
//   public void resetEncoders() {
//     m_drivingEncoder.setPosition(0);
//   }

   

   
    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getAbsoluteAngle().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        m_turningSparkMax.getEncoder().setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        m_turningSparkMax.setInverted(Constants.Swerve.angleMotorInvert);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        m_drivingSparkMax.setInverted(Constants.Swerve.driveMotorInvert);
    }

}
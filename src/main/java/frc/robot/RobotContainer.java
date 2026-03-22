// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import edu.wpi.first.wpilibj.XboxController;
import swervelib.SwerveInputStream;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.*;
import frc.robot.commands.swerve.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

        // initialize subsystems
        private final SwerveSubsystem driveSubsystem = new SwerveSubsystem();

        // Initialize Controllers
        private final XboxController driverController = new XboxController(DriverConstants.MAIN_DRIVER_PORT);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                configureBindings();

                driveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

        }

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveSubsystem.getSwerveDrive(),
                        () -> driverController.getLeftY() * getMultiplier(),
                        () -> driverController.getLeftX() * getMultiplier())
                        .withControllerRotationAxis(() -> getRightX())
                        .deadband(getDeadzone())
                        .scaleTranslation(1)// Can be changed to alter speed
                        .allianceRelativeControl(false);

        SwerveInputStream driveRobotOrientedVelocity = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        Command driveFieldOrientedAngularVelocity = driveSubsystem.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOriented = driveSubsystem.driveFieldOriented(driveRobotOrientedVelocity);

        private void configureBindings() {

                // ----------------------- MAIN DRIVER CONTROLS
                // ----------------------------------------------------------------------

                // Left bumper causes straightforward driving
                new Trigger(driverController::getRightBumperButton).whileTrue(driveRobotOriented);

                // B button causes the driver to zero their controller
                new JoystickButton(driverController, XboxController.Button.kB.value)
                                .whileTrue(new ZeroGyro(driveSubsystem)); // zero gyro on B

        }

        private double getDeadzone() {
                return DriverConstants.DEADBAND;
        }

        private double getMultiplier() {
                if (driverController.getLeftStickButton()) {
                        return 1;
                } else if (getLeftDriverTriggerValue()) {
                        return 0.4;
                }
                return 0.5;
        }

        private double getTurnMultiplier() {
                if (driverController.getRightStickButton()) {
                        return 1;
                } else {
                        return 0.5;
                }
        }

        private boolean getLeftDriverTriggerValue() {
                if (driverController != null) {
                        if (driverController.getLeftTriggerAxis() >= 0.5) {
                                return true;
                        }
                        return false;
                } else {
                        return false;
                }
        }

        /**
         * 
         * @return Will return the controller input either divided by two or not based
         *         on whether you hold the joystick button. If you hold left trigger, it
         *         will use the limelight to auto rotate
         */
        private double getRightX() {
                // SmartDashboard.putNumber("pos rot",
                // driveSubsystem.getSwerveDrive().getPose().getRotation().getDegrees());
                if (getLeftDriverTriggerValue()) {

                        return getControllerRotation();

                }

                // System.out.println("switch: " + getSwitch());
                return getControllerRotation();
        }

        private double getControllerRotation() {
                return driverController.getRightX() * getTurnMultiplier();
        }

        public Command getAutonomousCommand() {
                return null;
        }
}

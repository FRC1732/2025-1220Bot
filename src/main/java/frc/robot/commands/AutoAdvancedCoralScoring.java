// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;

// /* You should consider using the more terse Command factories API instead
// https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class AutoAdvancedCoralScoring extends Command {
//   /** Creates a new AutoAdvancedCoralScoring. */
//   public AutoAdvancedCoralScoring() {
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralScoring;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAdvancedCoralScoring extends Command {
  /** Creates a new AdvancedCoralScoring. */
  private CoralScoring coralScoring;

  private final double reverseDelaySec = .45; // How long motor runs backwards
  private final double scoringDurationSec = .75; // How long motor runs fowrards
  private Timer timer;

  public AutoAdvancedCoralScoring(CoralScoring coralScoring) {
    this.timer = new Timer();
    this.coralScoring = coralScoring;
    addRequirements(coralScoring);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elapsed = timer.get();
    if (elapsed < reverseDelaySec) {
      coralScoring.reverse();
    } else if (elapsed < reverseDelaySec + scoringDurationSec) {
      coralScoring.scores();
    } else {
      // Optionally, you can stop the scoring or do something else after the scoring duration
      coralScoring.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralScoring.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= reverseDelaySec + scoringDurationSec;
  }
}

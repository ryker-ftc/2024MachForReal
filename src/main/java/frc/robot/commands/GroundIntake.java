// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Intaker;

// public class IntakeIn extends CommandBase {
//     Intaker m_Intaker;

//     public IntakeIn(Intaker intake) {
//         m_Intaker = intake;
//         addRequirements(m_Intaker);
//     }

//     @Override
//     public void execute() {
//         m_Intaker.pull();
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         m_Intaker.stop();
//     }
// }

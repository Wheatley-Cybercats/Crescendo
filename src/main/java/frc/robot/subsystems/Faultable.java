package frc.robot.subsystems;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.concurrent.*;
import lombok.Getter;

public class Faultable {
  @Getter public static ArrayList<Faultable> faultableList = new ArrayList<>();
  private Callable<Boolean> toTrigger;
  private Runnable whenTriggered;
  private long sleepInterval; // Interval in milliseconds for sleep between executions of toTrigger
  private ExecutorService executor;
  private static DecimalFormat df = new DecimalFormat("#.##");

  public Faultable(Callable<Boolean> toTrigger, Runnable whenTriggered, long sleepInterval) {
    this.toTrigger = toTrigger;
    this.whenTriggered = whenTriggered;
    this.sleepInterval = sleepInterval;
    this.executor = Executors.newFixedThreadPool(30);
  }

  public void execute() {
    executor.submit(
        () -> {
          try {
            while (!Thread.currentThread().isInterrupted()) {
              boolean shouldContinue = toTrigger.call();
              if (!shouldContinue) break; // Exit the loop based on the condition
              Thread.sleep(sleepInterval);
            }
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          } catch (Exception e) {
            // Handle exception
          } finally {
            // Ensure this is non-blocking
            whenTriggered.run();
          }
        });
    // Consider removing or adjusting blocking calls here
    // executor.shutdown(), awaitTermination(), etc. are removed or adjusted
  }

  public static String generateDebugMessage(double c1, double c2) {
    return " " + df.format(c1) + " | " + df.format(c2);
  }
}

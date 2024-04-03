package frc.robot.Subsystems;

import java.util.concurrent.*;

public class Faultable {
  private Callable<Boolean> toTrigger;
  private Runnable whenTriggered;
  private long sleepInterval; // Interval in milliseconds for sleep between executions of toTrigger
  private ExecutorService executor;

  public Faultable(Callable<Boolean> toTrigger, Runnable whenTriggered, long sleepInterval) {
    this.toTrigger = toTrigger;
    this.whenTriggered = whenTriggered;
    this.sleepInterval = sleepInterval;
    this.executor = Executors.newFixedThreadPool(2);
  }

  public void execute() {
    Future<?> future =
        executor.submit(
            () -> {
              try {
                boolean shouldContinue = true;
                while (!Thread.currentThread().isInterrupted() && shouldContinue) {
                  shouldContinue = toTrigger.call();
                  if (shouldContinue) {
                    Thread.sleep(sleepInterval); // Delay between iterations
                  }
                }
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Handle the interruption status
              } catch (Exception e) {
                // Handle other exceptions from the callable task
              } finally {
                // Ensure the whenTriggered runs after breaking out of the loop
                whenTriggered.run();
              }
            });
    try {
      future.get(); // Wait for task completion
    } catch (InterruptedException | ExecutionException e) {
      Thread.currentThread().interrupt();
    }
    executor.shutdown();
    try {
      if (!executor.awaitTermination(60, TimeUnit.SECONDS)) {
        executor.shutdownNow();
      }
    } catch (InterruptedException e) {
      executor.shutdownNow();
    }
  }
}

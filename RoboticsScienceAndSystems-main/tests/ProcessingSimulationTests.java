import processing.core.*;
import java.util.*;
import java.io.*;

/** Runs the actual Processing-compiled sketch's course physics and controller. */
public class ProcessingSimulationTests {
  static final double[][][] ROUTES={
    {{0,0},{300,0}},
    {{0,0},{200,0},{200,180}},
    {{0,0},{200,0},{200,160},{0,160},{0,0}},
    {{0,0},{100,40},{180,-40},{260,40},{340,0}},
    {{0,0},{180,180},{0,180},{180,0},{300,0}},
    {{0,0},{300,0}},
    {{0,0},{0,300}},
    {{0,0},{0,0},{150,0},{150,0},{150,150}}
  };
  static final String[] NAMES={"straight","right_angle","closed_square","s_curve","crossing","reverse","rotate_start","duplicates"};
  public static void main(String[] args) throws Exception {
    File folder=new File(args[0]);folder.mkdirs();
    int successes=0;
    try(PrintWriter summary=new PrintWriter(new File(folder,"rollouts.csv"))) {
      summary.println("route,success,terminal,seconds,goal_error_mm,remaining_mm,reason");
      for(int test=0;test<ROUTES.length;test++) {
        DigitalTwin app=new DigitalTwin();
        app.g=new processing.awt.PGraphicsJava2D();
        app.g.setParent(app);app.g.setSize(1,1);
        PImage image=new PImage(420,297,PConstants.RGB);
        Arrays.fill(image.pixels,0xffffffff);
        DigitalTwin.RobotSimulation_c robot=app.new RobotSimulation_c(16,90,image,135,0);
        DigitalTwin.Controller_c control=app.new Controller_c();
        robot.resetForRun(0,0,test==5?(float)Math.PI:0,0,0);
        ArrayList<PVector> points=new ArrayList<>();
        for(double[] p:ROUTES[test]) points.add(new PVector((float)p[0],(float)p[1]));
        if(!control.loadRoute(points)) throw new AssertionError(control.status);
        control.setSignal(1);
        long step=0;
        try(PrintWriter trace=new PrintWriter(new File(folder,NAMES[test]+".csv"))) {
          trace.println("t_ms,x_mm,y_mm,yaw,left_pwm,right_pwm,action,progress");
          while(control.running && step<30000) {
            robot.advanceTime(10);control.update(robot);robot.updatePhysics(10);step++;
            if(Math.abs(robot.motors.left.pwm)>100 || Math.abs(robot.motors.right.pwm)>100)
              throw new AssertionError("PWM bounds");
            if(step%5==0) trace.printf(Locale.US,"%d,%.9f,%.9f,%.9f,%.0f,%.0f,%s,%d%n",
              robot.getMillis(),robot.odometry.pose.x,robot.odometry.pose.y,robot.odometry.pose.theta,
              robot.motors.left.pwm,robot.motors.right.pwm,control.decision.name(),control.decision.progress);
          }
        }
        double[] end=ROUTES[test][ROUTES[test].length-1];
        double error=Math.hypot(robot.odometry.pose.x-end[0],robot.odometry.pose.y-end[1]);
        boolean success=!control.running && control.decision.terminal && error<=5.1;
        if(success) successes++;
        String row=String.format(Locale.US,"%s,%b,%b,%.2f,%.6f,%.6f,%s",NAMES[test],success,
          control.decision.terminal,step/100.0,error,control.decision.features[8]*1000,
          control.running?"timeout":control.status);
        summary.println(row);System.out.println(row);
        control.reset();control.update(robot);robot.setMotorPWM(0,0);
        float beforeX=robot.odometry.pose.x,beforeY=robot.odometry.pose.y;
        robot.updatePhysics(10);
        if(beforeX!=robot.odometry.pose.x || beforeY!=robot.odometry.pose.y) throw new AssertionError("STOP moved robot");
      }
    }
    System.out.println("Geometric endpoint success: "+successes+"/"+ROUTES.length);
    if(successes!=ROUTES.length) System.exit(2);
  }
}

/** Same frozen trees and 50 ms motor decision cycle as the C++ robot. */
class Controller_c {
  final int UPDATE_INTERVAL_MS=10, MODEL_INTERVAL_MS=50;
  TwoTreeController core=new TwoTreeController();
  TwoTreeController.Decision decision=new TwoTreeController.Decision();
  int signal=0;
  boolean running=false;
  long lastUpdateMs=0,lastModelMs=0;
  String status="Load a route to simulate";
  void reset() {
    signal=0; running=false; core.reset();
    decision=new TwoTreeController.Decision();
    lastUpdateMs=lastModelMs=0;
    status=core.ready()?"Ready":"Load a route to simulate";
  }
  boolean loadRoute(ArrayList<PVector> points) {
    reset();
    double[][] route=new double[points.size()][2];
    for(int i=0;i<points.size();i++) {
      route[i][0]=points.get(i).x/1000.0d;route[i][1]=points.get(i).y/1000.0d;
    }
    boolean loaded=core.loadPath(route);
    status=loaded?"Ready":core.error;return loaded;
  }
  int getSignal() { return signal; }
  void setSignal(int requested) {
    if(requested==0) { reset(); return; }
    if(core.ready() && !running) { core.reset();signal=1;running=true;status="Running"; }
  }
  void update(RobotSimulation_c robot) {
    long now=robot.getMillis();
    if(now-lastUpdateMs<UPDATE_INTERVAL_MS) return;
    lastUpdateMs=now;
    robot.getSurfaceSensors();robot.getEncoders();robot.getPose();
    if(!running) { robot.setMotorPWM(0,0);return; }
    if(now-lastModelMs<MODEL_INTERVAL_MS) return;
    lastModelMs=now;
    RobotPose_c p=robot.odometry.pose;
    decision=core.step(p.x/1000.0d,p.y/1000.0d,p.theta,
      (int)robot.motors.left.pwm,(int)robot.motors.right.pwm);
    if(!decision.valid) { running=false;status=core.error;robot.setMotorPWM(0,0);return; }
    robot.setMotorPWM(decision.left,decision.right);
    if(decision.action==TreeModels.STOP) {
      running=false;
      status=decision.terminal?"Reached endpoint":"Tree1 STOP before geometric endpoint";
    } else status=decision.name();
  }
}

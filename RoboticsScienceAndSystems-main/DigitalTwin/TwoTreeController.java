/** Java port of the deployed Tree1 geometry and Tree2 motor adapter.
 * SI units here; the course simulation converts its millimetre pose at entry. */
public final class TwoTreeController {
  public static final int CAPACITY=4096;
  private final double[] px=new double[CAPACITY], py=new double[CAPACITY], arc=new double[CAPACITY];
  private int count=0, progress=0;
  private double previousX, previousY, credit;
  private boolean havePrevious=false;
  public String error="no path loaded";
  public static final class Decision {
    public int action=TreeModels.STOP, left=0, right=0, progress=0, target=0;
    public double angle=0, targetX=0, targetY=0;
    public double[] features=new double[11];
    public boolean terminal=false, valid=false;
    public String name() { return new String[]{"FORWARD","REVERSE","ROTATE","STOP"}[action]; }
  }
  private static double clamp(double x,double lo,double hi) { return Math.max(lo,Math.min(hi,x)); }
  private static double wrap(double a) {
    a=(a+Math.PI)%(2*Math.PI); if(a<0) a+=2*Math.PI; return a-Math.PI;
  }
  private static int pwm(double x) {
    x=clamp(x,-100,100);
    return (int)Math.copySign(Math.floor(Math.abs(x)+0.5),x);
  }
  public void reset() { progress=0; havePrevious=false; credit=0; }
  public boolean ready() { return count>=2; }
  public int sampleCount() { return count; }
  private boolean fail(String reason) { count=0; reset(); error=reason; return false; }
  private boolean append(double x,double y) {
    if(count>=CAPACITY) return fail("path exceeds sample capacity");
    arc[count]=count==0?0:arc[count-1]+Math.hypot(x-px[count-1],y-py[count-1]);
    px[count]=x; py[count]=y; count++; return true;
  }
  public boolean loadPath(double[][] points) {
    count=0; reset();
    if(points==null || points.length<2 || points.length>256) return fail("path needs 2..256 points");
    double total=0;
    for(int i=0;i<points.length;i++) {
      if(points[i]==null || points[i].length!=2 || !Double.isFinite(points[i][0]) ||
         !Double.isFinite(points[i][1]) || Math.abs(points[i][0])>10 || Math.abs(points[i][1])>10)
        return fail("invalid coordinate");
      if(i>0) total+=Math.hypot(points[i][0]-points[i-1][0],points[i][1]-points[i-1][1]);
    }
    if(total>4.0+1e-9) return fail("route exceeds 4 metres");
    double x=points[0][0],y=points[0][1],next=TreeModels.SPACING;
    append(x,y); boolean distinct=false;
    for(int i=1;i<points.length;i++) {
      double bx=points[i][0],by=points[i][1],length=Math.hypot(bx-x,by-y);
      if(length<=1e-12) continue;
      distinct=true; double travelled=0;
      while(travelled+next<=length) {
        travelled+=next; double t=travelled/length;
        if(!append(x+t*(bx-x),y+t*(by-y))) return false;
        next=TreeModels.SPACING;
      }
      next-=length-travelled; x=bx; y=by;
    }
    if(!distinct) return fail("path needs distinct points");
    if(Math.hypot(px[count-1]-x,py[count-1]-y)>1e-12 && !append(x,y)) return false;
    error=""; return ready();
  }
  public Decision step(double x,double y,double yaw,int prevL,int prevR) {
    Decision out=new Decision();
    if(!ready() || !Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(yaw)) {
      error="invalid path or pose"; return out;
    }
    double length=arc[count-1];
    double margin=TreeModels.SPACING<length/2?2*TreeModels.SPACING:length;
    double limit=Math.min(Math.max(TreeModels.LOOKAHEAD,margin),length);
    double allowance=havePrevious?credit+2*Math.hypot(x-previousX,y-previousY):limit;
    if(!Double.isFinite(allowance)) { error="pose movement overflow";return out; }
    int nextProgress=progress,end=Math.min(count,progress+TreeModels.SEARCH_WINDOW+1);
    double best=Double.POSITIVE_INFINITY;
    for(int i=progress;i<end;i++) {
      if(havePrevious && arc[i]-arc[progress]>allowance+1e-12) break;
      double dx=px[i]-x,dy=py[i]-y,d=dx*dx+dy*dy;
      if(!Double.isFinite(d)) { error="pose/path distance overflow";return out; }
      if(!havePrevious && d>best) break;
      if(d<best) { best=d; nextProgress=i; }
    }
    int target=count-1; double travelled=0;
    for(int i=nextProgress+1;i<count;i++) {
      travelled+=Math.hypot(px[i]-px[i-1],py[i]-py[i-1]);
      if(travelled+1e-12>=TreeModels.LOOKAHEAD) { target=i; break; }
    }
    double vx=px[target]-x,vy=py[target]-y,d=Math.hypot(vx,vy);
    int ti=Math.min(nextProgress,count-2);
    double tx=px[ti+1]-px[ti],ty=py[ti+1]-py[ti],len=Math.hypot(tx,ty);
    if(len<=1e-12) { tx=1;ty=0; } else { tx/=len;ty/=len; }
    double heading=-yaw,tangent=-Math.atan2(ty,tx),tangentError=wrap(tangent-heading);
    boolean reverse=Math.abs(tangentError)>Math.PI/2;
    double travel=d>1e-12?-Math.atan2(vy,vx):tangent;
    double angle=wrap(travel+(reverse?Math.PI:0)-heading);
    double goal=Math.hypot(x-px[count-1],y-py[count-1]),remaining=arc[count-1]-arc[nextProgress];
    out.features=new double[]{vx,vy,d,tx,ty,angle,tangentError,goal,remaining,Math.abs(angle),Math.abs(tangentError)};
    for(double f:out.features) if(!Double.isFinite(f)) { error="invalid geometry"; return out; }
    out.action=TreeModels.action(out.features);
    out.terminal=goal<=TreeModels.GOAL_TOLERANCE && remaining<=TreeModels.GOAL_TOLERANCE;
    if(out.terminal) out.action=TreeModels.STOP;
    reverse=out.action==TreeModels.REVERSE || (out.action==TreeModels.ROTATE && reverse);
    out.angle=out.action==TreeModels.STOP?0:wrap(wrap(travel+(reverse?Math.PI:0))-heading);
    out.progress=nextProgress;out.target=target;out.targetX=px[target];out.targetY=py[target];
    if(!havePrevious) credit=limit;
    else credit=clamp(allowance-(arc[nextProgress]-arc[progress]),0,limit);
    previousX=x;previousY=y;havePrevious=true;progress=nextProgress;
    out.valid=motors(out,prevL,prevR); error=out.valid?"":"invalid motor input";
    return out;
  }
  private boolean motors(Decision out,int prevL,int prevR) {
    if(out.action==TreeModels.STOP) return true;
    double a=out.angle%(2*Math.PI);
    if(a<=-Math.PI) a+=2*Math.PI; if(a>Math.PI) a-=2*Math.PI;
    if(out.action==TreeModels.ROTATE) {
      double turn=clamp(45*a,-35,35);out.left=pwm(turn);out.right=pwm(-turn);return true;
    }
    if(Math.abs(prevL)>100 || Math.abs(prevR)>100) return false;
    int signal=out.action==TreeModels.FORWARD?1:-1;
    float difference=TreeModels.difference((float)a,signal,prevL,prevR);
    if(!Float.isFinite(difference)) return false;
    double base=(signal>0?45:-35)*clamp(Math.cos(a),0,1)
      *clamp(Math.max(out.features[7],out.features[8])/TreeModels.LOOKAHEAD,0,1);
    if(signal>0) base*=TreeModels.forwardSpeedScale((float)a,difference);
    double limit=2*(100-Math.abs(base)),diff=clamp(difference,-limit,limit);
    out.left=pwm(base-diff/2);out.right=pwm(base+diff/2);return true;
  }
}

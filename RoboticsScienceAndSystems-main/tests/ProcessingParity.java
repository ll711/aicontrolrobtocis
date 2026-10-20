import java.util.*;
public class ProcessingParity {
  public static void main(String[] args) {
    Scanner input=new Scanner(System.in).useLocale(Locale.US);
    TwoTreeController controller=new TwoTreeController();
    while(input.hasNext()) {
      String kind=input.next();
      if(kind.equals("R")) {
        double[][] path=new double[input.nextInt()][2];
        for(double[] p:path) {p[0]=input.nextDouble();p[1]=input.nextDouble();}
        if(!controller.loadPath(path)) throw new AssertionError(controller.error);
      } else {
        TwoTreeController.Decision d=controller.step(input.nextDouble(),input.nextDouble(),input.nextDouble(),input.nextInt(),input.nextInt());
        System.out.printf(Locale.US,"%d %d %d %.17g %d %d %d",d.action,d.progress,d.target,d.angle,d.left,d.right,d.terminal?1:0);
        for(double f:d.features) System.out.printf(Locale.US," %.17g",f);
        System.out.println();
      }
    }
  }
}
